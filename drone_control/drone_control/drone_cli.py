#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Command line client of drone_commander (one drone).

  ros2 run drone_control drone_cli drone1 state [--watch]
  ros2 run drone_control drone_cli drone1 takeoff 10
  ros2 run drone_control drone_cli drone1 goto 20 10 15 [--yaw 90] [--local]
  ros2 run drone_control drone_cli drone1 vel 2 0 0 [--yaw-rate 10] [--body] [--duration 5]
  ros2 run drone_control drone_cli drone1 global -35.3632 149.1652 20 [--yaw 0]
  ros2 run drone_control drone_cli drone1 land | rtl | hold | arm | disarm
  ros2 run drone_control drone_cli drone1 mode LOITER

Angles in degrees (yaw ENU: 0 = East, 90 = North). Positions in the swarm
frame "map" (ENU) unless --local (local EKF frame of the drone).
Works in the domain of the drone (ROS_DOMAIN_ID = id) or from the GCS domain
through the Zenoh bridge.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from drone_interfaces.msg import DroneState, GlobalTarget
from drone_interfaces.srv import SetMode, Takeoff

SETPOINT_NAMES = {0: 'none', 1: 'velocity', 2: 'position', 3: 'global'}


def format_state(s):
    pos = (f'({s.position.x:7.2f}, {s.position.y:7.2f}, {s.position.z:6.2f}) m  yaw {math.degrees(s.yaw):6.1f} deg'
           if s.position_valid else 'position not valid')
    vel = math.sqrt(s.velocity.x ** 2 + s.velocity.y ** 2 + s.velocity.z ** 2)
    bat = f'{s.battery_voltage:.1f} V' + ('' if math.isnan(s.battery_remaining) else f' {100 * s.battery_remaining:.0f}%')
    return (f'{s.name}: {"CONNECTED" if s.connected else "NO FCU"} | {"ARMED" if s.armed else "disarmed"} | '
            f'{s.mode:10s} | {pos} | |v| {vel:4.1f} m/s | alt {s.relative_altitude:5.1f} m | {bat} | '
            f'cmd {SETPOINT_NAMES.get(s.setpoint_type, "?")}')


class DroneCli(Node):

    def __init__(self, drone):
        super().__init__(f'drone_cli_{drone}_{int(time.time() * 1000) % 100000}')
        self.drone = drone.strip('/')

    def ns(self, name):
        return f'/{self.drone}/{name}'

    def wait_state(self, timeout=5.0):
        box = {}
        sub = self.create_subscription(DroneState, self.ns('state'), lambda m: box.setdefault('s', m), 10)
        t0 = time.monotonic()
        while 's' not in box and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return box.get('s')

    def call(self, srv_type, name, request, timeout=30.0):
        client = self.create_client(srv_type, self.ns(name))
        if not client.wait_for_service(timeout_sec=5.0):
            print(f'service {self.ns(name)} not available (is drone_commander running / routed by Zenoh?)')
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def publish(self, msg_type, topic, msg, repeat=3):
        pub = self.create_publisher(msg_type, self.ns(topic), 10)
        t0 = time.monotonic()
        while pub.get_subscription_count() == 0 and time.monotonic() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            print(f'nobody subscribed to {self.ns(topic)} (is drone_commander running?)')
            return False
        for _ in range(repeat):
            pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('drone', help='drone name / namespace, e.g. drone1')
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('state')
    p.add_argument('--watch', action='store_true')
    p = sub.add_parser('takeoff')
    p.add_argument('altitude', type=float)
    for name in ('land', 'rtl', 'hold', 'arm', 'disarm'):
        sub.add_parser(name)
    p = sub.add_parser('mode')
    p.add_argument('mode')
    p = sub.add_parser('goto')
    p.add_argument('x', type=float)
    p.add_argument('y', type=float)
    p.add_argument('z', type=float)
    p.add_argument('--yaw', type=float, default=None, help='deg ENU (default: keep heading)')
    p.add_argument('--local', action='store_true', help='local EKF frame instead of the swarm map frame')
    p = sub.add_parser('vel')
    p.add_argument('vx', type=float)
    p.add_argument('vy', type=float)
    p.add_argument('vz', type=float)
    p.add_argument('--yaw-rate', type=float, default=0.0, help='deg/s (counter-clockwise +)')
    p.add_argument('--body', action='store_true', help='body frame (x forward, y left)')
    p.add_argument('--duration', type=float, default=3.0)
    p = sub.add_parser('global')
    p.add_argument('lat', type=float)
    p.add_argument('lon', type=float)
    p.add_argument('alt', type=float, help='m above home')
    p.add_argument('--yaw', type=float, default=None, help='deg ENU (default: keep heading)')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = DroneCli(args.drone)
    code = 0
    try:
        if args.cmd == 'state':
            while True:
                s = node.wait_state()
                if s is None:
                    print(f'no state received on {node.ns("state")}')
                    code = 1
                    break
                print(format_state(s))
                if not args.watch:
                    break
                time.sleep(0.5)
        elif args.cmd == 'takeoff':
            r = node.call(Takeoff, 'takeoff', Takeoff.Request(altitude=args.altitude))
            print(r.message if r else 'no response')
            code = 0 if r and r.success else 1
        elif args.cmd in ('land', 'rtl', 'hold'):
            r = node.call(Trigger, args.cmd, Trigger.Request())
            print(r.message if r else 'no response')
            code = 0 if r and r.success else 1
        elif args.cmd in ('arm', 'disarm'):
            r = node.call(SetBool, 'arm', SetBool.Request(data=args.cmd == 'arm'))
            print(r.message if r else 'no response')
            code = 0 if r and r.success else 1
        elif args.cmd == 'mode':
            r = node.call(SetMode, 'set_mode', SetMode.Request(mode=args.mode))
            print(r.message if r else 'no response')
            code = 0 if r and r.success else 1
        elif args.cmd == 'goto':
            msg = PoseStamped()
            msg.header.frame_id = f'{node.drone}/odom' if args.local else 'map'
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = args.x, args.y, args.z
            if args.yaw is not None:
                yaw = math.radians(args.yaw)
                msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            else:
                msg.pose.orientation.w = 0.0   # all-zero quaternion = keep heading
            ok = node.publish(PoseStamped, 'cmd/position', msg)
            print(f'goto ({args.x}, {args.y}, {args.z}) [{msg.header.frame_id}] sent' if ok else 'not sent')
            code = 0 if ok else 1
        elif args.cmd == 'vel':
            pub = node.create_publisher(TwistStamped, node.ns('cmd/velocity'), 10)
            msg = TwistStamped()
            msg.header.frame_id = 'base_link' if args.body else 'map'
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = args.vx, args.vy, args.vz
            msg.twist.angular.z = math.radians(args.yaw_rate)
            t0 = time.monotonic()
            while time.monotonic() - t0 < args.duration:
                msg.header.stamp = node.get_clock().now().to_msg()
                pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.1)
            print(f'velocity sent for {args.duration:.1f} s (then the drone holds position)')
        elif args.cmd == 'global':
            msg = GlobalTarget()
            msg.latitude, msg.longitude, msg.altitude = args.lat, args.lon, args.alt
            msg.yaw = float('nan') if args.yaw is None else math.radians(args.yaw)
            ok = node.publish(GlobalTarget, 'cmd/global', msg)
            print('global target sent' if ok else 'not sent')
            code = 0 if ok else 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
