#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
drone_commander: high level interface between ROS 2 (decision making, vision,
AI, swarm) and the ArduPilot flight controller (through MAVROS).

ArduPilot keeps doing the flight control (attitude, velocity and position
loops, EKF, failsafes). This node only sends GUIDED mode targets. It runs in the
namespace of the drone (/<name>) and is identical in simulation and on the
real drone.

Commands (only executed in GUIDED mode, the last one received wins; during a
takeoff they are held and applied when the takeoff altitude is reached):
  cmd/velocity  geometry_msgs/TwistStamped   frame_id "map" (ENU) or "base_link" (body,
                                               heading-relative). angular.z = yaw rate.
                                               Must be streamed: after `velocity_timeout`
                                               without messages the drone holds position.
  cmd/position  geometry_msgs/PoseStamped    frame_id "map" (swarm frame) or "<name>/odom"
                                               (local EKF frame). Orientation = yaw target;
                                               all-zero quaternion = keep current heading.
  cmd/global    drone_interfaces/GlobalTarget lat / lon / altitude above home.
Services:
  takeoff (drone_interfaces/Takeoff)  land / rtl / hold (std_srvs/Trigger)
  arm (std_srvs/SetBool)              set_mode (drone_interfaces/SetMode)
Outputs:
  state (drone_interfaces/DroneState, 10 Hz)   pose (PoseStamped, frame "map")
  TF: map -> <name>/odom -> <name>/base_link
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from mavros_msgs.msg import GlobalPositionTarget, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, StreamRate
from mavros_msgs.srv import SetMode as MavSetMode
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

from drone_control import geo
from drone_interfaces.msg import DroneState, GlobalTarget
from drone_interfaces.srv import SetMode, Takeoff

# mavros_msgs/PositionTarget type_mask bits
_IGNORE_PX, _IGNORE_PY, _IGNORE_PZ = 1, 2, 4
_IGNORE_VX, _IGNORE_VY, _IGNORE_VZ = 8, 16, 32
_IGNORE_AFX, _IGNORE_AFY, _IGNORE_AFZ = 64, 128, 256
_IGNORE_YAW, _IGNORE_YAW_RATE = 1024, 2048
_MASK_VELOCITY = (_IGNORE_PX | _IGNORE_PY | _IGNORE_PZ | _IGNORE_AFX | _IGNORE_AFY | _IGNORE_AFZ | _IGNORE_YAW)
_MASK_POSITION = (_IGNORE_VX | _IGNORE_VY | _IGNORE_VZ | _IGNORE_AFX | _IGNORE_AFY | _IGNORE_AFZ | _IGNORE_YAW_RATE)

# MAV_DATA_STREAM ids and rates [Hz] requested to the FCU at connection
_STREAMS = {1: 2,     # RAW_SENSORS
            2: 2,     # EXTENDED_STATUS (SYS_STATUS, battery)
            3: 2,     # RC_CHANNELS
            6: 20,    # POSITION (LOCAL_POSITION_NED, GLOBAL_POSITION_INT)
            10: 20,   # EXTRA1 (ATTITUDE)
            11: 10,   # EXTRA2 (VFR_HUD)
            12: 2}    # EXTRA3 (EKF status, wind, vibration)

_BODY_FRAMES = ('base_link', 'body', 'flu')
_MAP_FRAMES = ('', 'map', 'enu', 'world')


class DroneCommander(Node):

    def __init__(self):
        super().__init__('drone_commander')

        ns = self.get_namespace().strip('/')
        self.declare_parameter('name', ns or 'drone1')
        self.declare_parameter('id', 0)
        # swarm frame origin (in simulation: the world <spherical_coordinates>)
        self.declare_parameter('origin_lat', float('nan'))
        self.declare_parameter('origin_lon', float('nan'))
        self.declare_parameter('origin_alt', 0.0)
        self.declare_parameter('setpoint_rate', 20.0)
        self.declare_parameter('state_rate', 10.0)
        self.declare_parameter('velocity_timeout', 0.5)
        self.declare_parameter('max_horizontal_speed', 10.0)
        self.declare_parameter('max_vertical_speed', 3.0)
        self.declare_parameter('max_yaw_rate', 1.5)
        self.declare_parameter('auto_guided', False)
        self.declare_parameter('request_streams', True)
        self.declare_parameter('publish_tf', True)

        self.name = self.get_parameter('name').value
        self.drone_id = int(self.get_parameter('id').value)
        self.origin = (self.get_parameter('origin_lat').value,
                       self.get_parameter('origin_lon').value,
                       self.get_parameter('origin_alt').value)
        self.vel_timeout = float(self.get_parameter('velocity_timeout').value)
        self.max_vh = float(self.get_parameter('max_horizontal_speed').value)
        self.max_vz = float(self.get_parameter('max_vertical_speed').value)
        self.max_wz = float(self.get_parameter('max_yaw_rate').value)
        self.auto_guided = bool(self.get_parameter('auto_guided').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.frame_odom = f'{self.name}/odom'
        self.frame_base = f'{self.name}/base_link'

        self.lock = threading.RLock()
        # FCU state
        self.mav_state = State()
        self.local_pose = None          # PoseStamped in <name>/odom
        self.local_pose_time = 0.0
        self.local_vel = None           # (vx, vy, vz) ENU
        self.fix = None                 # NavSatFix (EKF global position)
        self.fix_time = 0.0
        self.rel_alt = float('nan')
        self.battery = None
        # local EKF frame -> swarm map frame (translation only, both ENU)
        self.offset = None
        self.offset_samples = []
        # active command
        self.sp_type = DroneState.SETPOINT_NONE
        self.sp_target_map = None       # (x, y, z, yaw) position target in the map frame
        self.sp_velocity = None         # (vx, vy, vz, yaw_rate) ENU
        self.sp_global = None           # GlobalTarget
        self.sp_vel_time = 0.0
        self.guided_requested_at = -1e9
        self.takeoff_alt = None         # takeoff in progress: commands wait until this altitude
        self.takeoff_deadline = 0.0
        self.streams_requested = False
        self.last_warn = {}

        if math.isnan(self.origin[0]) or math.isnan(self.origin[1]):
            self.get_logger().warn('origin_lat / origin_lon not set: the swarm "map" frame origin will be '
                                   'the first GPS position of this drone (set them for a shared swarm frame)')

        sub_group = MutuallyExclusiveCallbackGroup()
        self.cli_group = ReentrantCallbackGroup()
        srv_group = ReentrantCallbackGroup()

        # ---------------- MAVROS ----------------
        self.create_subscription(State, 'mavros/state', self.on_state, 10, callback_group=sub_group)
        self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.on_local_pose,
                                 qos_profile_sensor_data, callback_group=sub_group)
        self.create_subscription(TwistStamped, 'mavros/local_position/velocity_local', self.on_local_vel,
                                 qos_profile_sensor_data, callback_group=sub_group)
        self.create_subscription(NavSatFix, 'mavros/global_position/global', self.on_fix,
                                 qos_profile_sensor_data, callback_group=sub_group)
        self.create_subscription(Float64, 'mavros/global_position/rel_alt', self.on_rel_alt,
                                 qos_profile_sensor_data, callback_group=sub_group)
        self.create_subscription(BatteryState, 'mavros/battery', self.on_battery,
                                 qos_profile_sensor_data, callback_group=sub_group)
        self.pub_raw_local = self.create_publisher(PositionTarget, 'mavros/setpoint_raw/local', 10)
        self.pub_raw_global = self.create_publisher(GlobalPositionTarget, 'mavros/setpoint_raw/global', 10)

        self.cli_set_mode = self.create_client(MavSetMode, 'mavros/set_mode', callback_group=self.cli_group)
        self.cli_arming = self.create_client(CommandBool, 'mavros/cmd/arming', callback_group=self.cli_group)
        self.cli_takeoff = self.create_client(CommandTOL, 'mavros/cmd/takeoff', callback_group=self.cli_group)
        self.cli_stream = self.create_client(StreamRate, 'mavros/set_stream_rate', callback_group=self.cli_group)

        # ---------------- high level API ----------------
        self.create_subscription(TwistStamped, 'cmd/velocity', self.on_cmd_velocity, 10, callback_group=sub_group)
        self.create_subscription(PoseStamped, 'cmd/position', self.on_cmd_position, 10, callback_group=sub_group)
        self.create_subscription(GlobalTarget, 'cmd/global', self.on_cmd_global, 10, callback_group=sub_group)
        self.pub_state = self.create_publisher(DroneState, 'state', 10)
        self.pub_pose = self.create_publisher(PoseStamped, 'pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_service(Takeoff, 'takeoff', self.srv_takeoff, callback_group=srv_group)
        self.create_service(Trigger, 'land', self.srv_land, callback_group=srv_group)
        self.create_service(Trigger, 'rtl', self.srv_rtl, callback_group=srv_group)
        self.create_service(Trigger, 'hold', self.srv_hold, callback_group=srv_group)
        self.create_service(SetBool, 'arm', self.srv_arm, callback_group=srv_group)
        self.create_service(SetMode, 'set_mode', self.srv_set_mode, callback_group=srv_group)

        self.create_timer(1.0 / float(self.get_parameter('setpoint_rate').value), self.control_loop,
                          callback_group=sub_group)
        self.create_timer(1.0 / float(self.get_parameter('state_rate').value), self.publish_state,
                          callback_group=sub_group)
        if self.get_parameter('request_streams').value:
            self.create_timer(2.0, self.request_streams, callback_group=self.cli_group)

        self.get_logger().info(f'[{self.name}] drone_commander ready (namespace {self.get_namespace()})')

    # ======================================================================
    # helpers
    # ======================================================================
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def warn_throttle(self, key, msg, period=5.0):
        t = time.monotonic()
        if t - self.last_warn.get(key, -1e9) > period:
            self.last_warn[key] = t
            self.get_logger().warn(msg)

    def call(self, client, request, timeout=5.0):
        """Synchronous call usable from service callbacks (MultiThreadedExecutor)."""
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            return None
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            return None
        return future.result()

    def current_yaw(self):
        q = self.local_pose.pose.orientation
        return geo.yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def position_valid(self):
        t = self.now()
        return (self.offset is not None and self.local_pose is not None and
                t - self.local_pose_time < 1.0 and t - self.fix_time < 2.0)

    def local_to_map(self, x, y, z):
        return x + self.offset[0], y + self.offset[1], z + self.offset[2]

    def map_to_local(self, x, y, z):
        return x - self.offset[0], y - self.offset[1], z - self.offset[2]

    # ======================================================================
    # MAVROS callbacks
    # ======================================================================
    def on_state(self, msg):
        with self.lock:
            prev = self.mav_state
            self.mav_state = msg
            if not msg.connected:
                self.streams_requested = False
            if msg.mode != prev.mode:
                # A mode change made outside this node (RC, QGC, failsafe) cancels the ROS command:
                # coming back to GUIDED later must not resume an old target.
                requested = msg.mode == 'GUIDED' and self.now() - self.guided_requested_at < 5.0
                if not requested and self.sp_type != DroneState.SETPOINT_NONE:
                    self.get_logger().info(f'[{self.name}] mode {prev.mode} -> {msg.mode}: ROS command cancelled')
                    self.sp_type = DroneState.SETPOINT_NONE

    def on_local_pose(self, msg):
        with self.lock:
            self.local_pose = msg
            self.local_pose_time = self.now()
            if self.offset is None:
                return
            p = msg.pose.position
            x, y, z = self.local_to_map(p.x, p.y, p.z)
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
            pose.pose.orientation = msg.pose.orientation
        self.pub_pose.publish(pose)
        if self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.frame_odom
            t.child_frame_id = self.frame_base
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
            t.transform.rotation = msg.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def on_local_vel(self, msg):
        v = msg.twist.linear
        with self.lock:
            self.local_vel = (v.x, v.y, v.z)

    def on_fix(self, msg):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        with self.lock:
            self.fix = msg
            self.fix_time = self.now()
            if math.isnan(self.origin[0]) or math.isnan(self.origin[1]):
                self.origin = (msg.latitude, msg.longitude, self.origin[2])
                self.get_logger().info(f'[{self.name}] swarm origin = first fix {msg.latitude:.7f}, {msg.longitude:.7f}')
            self.update_offset()

    def update_offset(self):
        """Offset local EKF frame -> map frame, from simultaneous global and local positions.
        Horizontal only: z of the map frame = height above the take-off point of the drone."""
        if self.local_pose is None or self.now() - self.local_pose_time > 0.5:
            return
        # same flat-earth model as the ArduPilot EKF (see geo.py): the offset is a constant translation
        e, n = geo.ap_latlon_to_en(self.fix.latitude, self.fix.longitude, self.origin[0], self.origin[1])
        p = self.local_pose.pose.position
        # time-align the local position to the fix (both stamped with FCU-synchronised time)
        dt = (rclpy.time.Time.from_msg(self.fix.header.stamp) -
              rclpy.time.Time.from_msg(self.local_pose.header.stamp)).nanoseconds * 1e-9
        if abs(dt) > 0.5:
            return
        vx, vy = (self.local_vel[0], self.local_vel[1]) if self.local_vel is not None else (0.0, 0.0)
        sample = (e - (p.x + vx * dt), n - (p.y + vy * dt))
        if self.offset is None:
            self.offset_samples.append(sample)
            if len(self.offset_samples) >= 10:
                ox = sum(s[0] for s in self.offset_samples) / len(self.offset_samples)
                oy = sum(s[1] for s in self.offset_samples) / len(self.offset_samples)
                self.offset = (ox, oy, 0.0)
                self.get_logger().info(f'[{self.name}] local EKF origin at map ({ox:.2f}, {oy:.2f}): position valid')
        else:
            # slow refinement, only when (almost) stopped: avoids the bias of non simultaneous samples
            if self.local_vel is not None and math.hypot(self.local_vel[0], self.local_vel[1]) < 1.0:
                a = 0.01
                self.offset = ((1 - a) * self.offset[0] + a * sample[0],
                               (1 - a) * self.offset[1] + a * sample[1], 0.0)

    def on_rel_alt(self, msg):
        with self.lock:
            self.rel_alt = msg.data

    def on_battery(self, msg):
        with self.lock:
            self.battery = msg

    # ======================================================================
    # high level commands
    # ======================================================================
    def accept_command(self, kind):
        """True if a command can be executed now (GUIDED), optionally switching to GUIDED."""
        with self.lock:
            st = self.mav_state
            if not st.connected:
                self.warn_throttle('nc', f'[{self.name}] {kind} ignored: FCU not connected')
                return False
            if not self.position_valid():
                self.warn_throttle('np', f'[{self.name}] {kind} ignored: no position estimate (EKF / GPS)')
                return False
            if st.mode == 'GUIDED':
                return True
            if self.auto_guided and st.armed:
                self.guided_requested_at = self.now()
                req = MavSetMode.Request(base_mode=0, custom_mode='GUIDED')
                self.cli_set_mode.call_async(req)
                self.warn_throttle('ag', f'[{self.name}] {kind}: switching {st.mode} -> GUIDED (auto_guided)')
                return True
            self.warn_throttle('ng', f'[{self.name}] {kind} ignored: mode {st.mode} '
                                     '(commands are executed in GUIDED: takeoff / hold / set_mode services)')
            return False

    def on_cmd_velocity(self, msg):
        if not self.accept_command('cmd/velocity'):
            return
        frame = msg.header.frame_id.split('/')[-1].lower()
        vx, vy, vz = msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z
        wz = msg.twist.angular.z
        with self.lock:
            if frame in _BODY_FRAMES:
                yaw = self.current_yaw()
                vx, vy = vx * math.cos(yaw) - vy * math.sin(yaw), vx * math.sin(yaw) + vy * math.cos(yaw)
            elif frame not in _MAP_FRAMES and frame != 'odom':
                self.warn_throttle('vf', f'[{self.name}] cmd/velocity: unknown frame "{msg.header.frame_id}"')
                return
            vh = math.hypot(vx, vy)
            if vh > self.max_vh:
                vx, vy = vx * self.max_vh / vh, vy * self.max_vh / vh
            vz = max(-self.max_vz, min(self.max_vz, vz))
            wz = max(-self.max_wz, min(self.max_wz, wz))
            self.sp_velocity = (vx, vy, vz, wz)
            self.sp_vel_time = self.now()
            self.sp_type = DroneState.SETPOINT_VELOCITY

    def on_cmd_position(self, msg):
        if not self.accept_command('cmd/position'):
            return
        frame = msg.header.frame_id
        p, q = msg.pose.position, msg.pose.orientation
        with self.lock:
            if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
                yaw = self.current_yaw()            # keep heading
            else:
                yaw = geo.yaw_from_quaternion(q.x, q.y, q.z, q.w)
            if frame.lower() in _MAP_FRAMES:
                target = (p.x, p.y, p.z)
            elif frame in (self.frame_odom, 'odom', 'local'):
                target = self.local_to_map(p.x, p.y, p.z)
            else:
                self.warn_throttle('pf', f'[{self.name}] cmd/position: unknown frame "{frame}" '
                                         f'(use "map" or "{self.frame_odom}")')
                return
            self.sp_target_map = (target[0], target[1], target[2], yaw)
            self.sp_type = DroneState.SETPOINT_POSITION

    def on_cmd_global(self, msg):
        if not self.accept_command('cmd/global'):
            return
        with self.lock:
            if math.isnan(msg.yaw):
                msg.yaw = float(self.current_yaw())
            self.sp_global = msg
            self.sp_type = DroneState.SETPOINT_GLOBAL

    def hold_here(self):
        """Position target = current position and heading."""
        p = self.local_pose.pose.position
        x, y, z = self.local_to_map(p.x, p.y, p.z)
        self.sp_target_map = (x, y, z, self.current_yaw())
        self.sp_type = DroneState.SETPOINT_POSITION

    # ======================================================================
    # setpoint streaming (GUIDED)
    # ======================================================================
    def control_loop(self):
        with self.lock:
            st = self.mav_state
            if not st.connected or st.mode != 'GUIDED':
                self.takeoff_alt = None
                return
            if self.takeoff_alt is not None:
                # A GUIDED target sent during the takeoff replaces it (the drone stays on the
                # ground and auto-disarms): commands received meanwhile wait until it ends
                done = (not st.armed or self.now() > self.takeoff_deadline or
                        (not math.isnan(self.rel_alt) and self.rel_alt >= 0.9 * self.takeoff_alt))
                if not done:
                    return
                self.takeoff_alt = None
                self.get_logger().info(f'[{self.name}] takeoff complete')
            if self.sp_type == DroneState.SETPOINT_NONE or not self.position_valid():
                return
            stamp = self.get_clock().now().to_msg()

            if self.sp_type == DroneState.SETPOINT_VELOCITY:
                if self.now() - self.sp_vel_time > self.vel_timeout:
                    self.get_logger().info(f'[{self.name}] cmd/velocity timeout -> holding position')
                    self.hold_here()
                else:
                    vx, vy, vz, wz = self.sp_velocity
                    sp = PositionTarget()
                    sp.header.stamp = stamp
                    sp.header.frame_id = self.frame_odom
                    sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED   # MAVROS converts ENU -> NED
                    sp.type_mask = _MASK_VELOCITY
                    sp.velocity.x, sp.velocity.y, sp.velocity.z = vx, vy, vz
                    sp.yaw_rate = wz
                    self.pub_raw_local.publish(sp)
                    return

            if self.sp_type == DroneState.SETPOINT_POSITION:
                x, y, z, yaw = self.sp_target_map
                lx, ly, lz = self.map_to_local(x, y, z)
                sp = PositionTarget()
                sp.header.stamp = stamp
                sp.header.frame_id = self.frame_odom
                sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                sp.type_mask = _MASK_POSITION
                sp.position.x, sp.position.y, sp.position.z = lx, ly, lz
                sp.yaw = yaw
                self.pub_raw_local.publish(sp)
            elif self.sp_type == DroneState.SETPOINT_GLOBAL:
                g = self.sp_global
                sp = GlobalPositionTarget()
                sp.header.stamp = stamp
                sp.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
                sp.type_mask = _MASK_POSITION
                sp.latitude, sp.longitude, sp.altitude = g.latitude, g.longitude, float(g.altitude)
                sp.yaw = float(g.yaw)
                self.pub_raw_global.publish(sp)

    # ======================================================================
    # state / TF
    # ======================================================================
    def publish_state(self):
        with self.lock:
            st = self.mav_state
            msg = DroneState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.name = self.name
            msg.id = self.drone_id
            msg.connected = st.connected
            msg.armed = st.armed
            msg.guided = st.mode == 'GUIDED'
            msg.mode = st.mode
            msg.system_status = st.system_status
            msg.position_valid = self.position_valid()
            if msg.position_valid:
                p = self.local_pose.pose.position
                msg.position.x, msg.position.y, msg.position.z = self.local_to_map(p.x, p.y, p.z)
                msg.yaw = self.current_yaw()
            if self.local_vel is not None:
                msg.velocity.x, msg.velocity.y, msg.velocity.z = self.local_vel
            if self.fix is not None:
                msg.latitude, msg.longitude = self.fix.latitude, self.fix.longitude
            msg.relative_altitude = float(self.rel_alt)
            if self.battery is not None:
                msg.battery_voltage = float(self.battery.voltage)
                pct = self.battery.percentage
                msg.battery_remaining = float(pct) if pct is not None and pct >= 0.0 else float('nan')
            else:
                msg.battery_remaining = float('nan')
            msg.setpoint_type = self.sp_type
            if self.sp_type == DroneState.SETPOINT_POSITION and self.sp_target_map:
                msg.target.x, msg.target.y, msg.target.z = self.sp_target_map[:3]
            offset = self.offset
        self.pub_state.publish(msg)

        if self.tf_broadcaster and offset is not None:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'map'
            t.child_frame_id = self.frame_odom
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = offset
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)

    def request_streams(self):
        """Ask the FCU for the MAVLink streams on the MAVROS link (works on any ArduPilot version)."""
        with self.lock:
            if self.streams_requested or not self.mav_state.connected:
                return
            self.streams_requested = True
        ok = True
        for stream_id, rate in _STREAMS.items():
            res = self.call(self.cli_stream, StreamRate.Request(stream_id=stream_id, message_rate=rate, on_off=True), 3.0)
            ok = ok and res is not None
        if ok:
            self.get_logger().info(f'[{self.name}] MAVLink stream rates requested')
        else:
            with self.lock:
                self.streams_requested = False

    # ======================================================================
    # services
    # ======================================================================
    def set_mode(self, mode, timeout=4.0):
        mode = mode.upper()
        if mode == 'GUIDED':
            self.guided_requested_at = self.now()
        res = self.call(self.cli_set_mode, MavSetMode.Request(base_mode=0, custom_mode=mode))
        if res is None or not res.mode_sent:
            return False, f'set_mode {mode} not sent (MAVROS / FCU not available?)'
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.mav_state.mode == mode:
                return True, f'mode {mode}'
            time.sleep(0.05)
        return False, f'FCU did not switch to {mode} (current {self.mav_state.mode}); see the FCU messages in QGC'

    def arm(self, value, timeout=5.0):
        res = self.call(self.cli_arming, CommandBool.Request(value=value))
        if res is None or not res.success:
            return False, ('arming' if value else 'disarming') + ' rejected (pre-arm checks? see QGC messages)'
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.mav_state.armed == value:
                return True, 'armed' if value else 'disarmed'
            time.sleep(0.05)
        return False, 'arming state did not change'

    def srv_takeoff(self, request, response):
        alt = float(request.altitude)
        if alt <= 0.5:
            response.success, response.message = False, 'altitude must be > 0.5 m'
            return response
        if not self.mav_state.connected:
            response.success, response.message = False, 'FCU not connected'
            return response
        if not self.position_valid():
            response.success, response.message = False, 'no position estimate yet (EKF / GPS not ready)'
            return response

        ok, msg = self.set_mode('GUIDED')
        if not ok:
            response.success, response.message = False, msg
            return response

        flying = self.mav_state.armed and not math.isnan(self.rel_alt) and self.rel_alt > 0.5
        if flying:
            with self.lock:
                p = self.local_pose.pose.position
                x, y, _ = self.local_to_map(p.x, p.y, p.z)
                self.sp_target_map = (x, y, alt, self.current_yaw())
                self.sp_type = DroneState.SETPOINT_POSITION
            response.success, response.message = True, f'already flying: going to {alt:.1f} m'
            return response

        with self.lock:
            self.sp_type = DroneState.SETPOINT_NONE
        if not self.mav_state.armed:
            ok, msg = self.arm(True)
            if not ok:
                response.success, response.message = False, msg
                return response
        res = self.call(self.cli_takeoff, CommandTOL.Request(min_pitch=0.0, yaw=0.0, latitude=0.0,
                                                             longitude=0.0, altitude=alt))
        if res is None or not res.success:
            response.success, response.message = False, 'takeoff command rejected by the FCU'
            return response
        with self.lock:
            self.takeoff_alt = alt
            self.takeoff_deadline = self.now() + 15.0 + alt / 1.0
        response.success, response.message = True, f'taking off to {alt:.1f} m'
        self.get_logger().info(f'[{self.name}] {response.message} (new commands are applied when it reaches the altitude)')
        return response

    def srv_land(self, request, response):
        response.success, response.message = self.set_mode('LAND')
        return response

    def srv_rtl(self, request, response):
        response.success, response.message = self.set_mode('RTL')
        return response

    def srv_hold(self, request, response):
        if not self.position_valid():
            response.success, response.message = False, 'no position estimate'
            return response
        if self.mav_state.mode != 'GUIDED':
            ok, msg = self.set_mode('GUIDED')
            if not ok:
                response.success, response.message = False, msg
                return response
        with self.lock:
            self.hold_here()
            x, y, z = self.sp_target_map[:3]
        response.success, response.message = True, f'holding at map ({x:.1f}, {y:.1f}, {z:.1f})'
        return response

    def srv_arm(self, request, response):
        response.success, response.message = self.arm(bool(request.data))
        return response

    def srv_set_mode(self, request, response):
        response.success, response.message = self.set_mode(request.mode)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DroneCommander()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
