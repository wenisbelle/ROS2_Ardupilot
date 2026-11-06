#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Swarm command line (usually run in the GCS domain, the commands reach the
drones through the Zenoh bridges).

  ros2 run swarm_control swarm_cli state                          # all drones found
  ros2 run swarm_control swarm_cli -d 1 2 3 takeoff 10
  ros2 run swarm_control swarm_cli -d 1-4 formation circle --spacing 5 --alt 12 --center 20 0
  ros2 run swarm_control swarm_cli formation line --spacing 4 --alt 10 --heading 90
  ros2 run swarm_control swarm_cli -d 2 3 goto 30 10 15           # same target, offset by --spacing
  ros2 run swarm_control swarm_cli land | rtl | hold

-d / --drones (before the command): ids (1 2 3 or 1,2,3), ranges (1-4) or
names (drone1 alpha). Default: every drone that publishes /<name>/state.
Formations: line, column, circle, grid, v. Heading in degrees ENU (0 = East).
Altitude separation: --alt-step adds k * alt_step to the k-th drone (safer).
"""
import argparse
import math
import re
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_srvs.srv import Trigger

from drone_interfaces.msg import DroneState
from drone_interfaces.srv import Takeoff
from swarm_control import formations

SETPOINT_NAMES = {0: 'none', 1: 'vel', 2: 'pos', 3: 'global'}
COMMANDS = ('state', 'takeoff', 'land', 'rtl', 'hold', 'formation', 'goto')


def parse_drones(items):
    names = []
    for it in items or []:
        m = re.fullmatch(r'(\d+)-(\d+)', it)
        if m:
            names += [f'drone{i}' for i in range(int(m.group(1)), int(m.group(2)) + 1)]
        elif it.isdigit():
            names.append(f'drone{it}')
        else:
            names.append(it.strip('/'))
    return names


class SwarmCli(Node):

    def __init__(self):
        super().__init__(f'swarm_cli_{int(time.time() * 1000) % 100000}')
        self.states = {}
        self.subs = {}

    def discover(self, timeout=3.0):
        """Names of the drones publishing <name>/state (drone_interfaces/msg/DroneState)."""
        t0 = time.monotonic()
        found = set()
        while time.monotonic() - t0 < timeout:
            for topic, types in self.get_topic_names_and_types():
                m = re.fullmatch(r'/([^/]+)/state', topic)
                if m and 'drone_interfaces/msg/DroneState' in types:
                    found.add(m.group(1))
            rclpy.spin_once(self, timeout_sec=0.2)
        return sorted(found, key=lambda s: (len(s), s))

    def collect_states(self, names, timeout=4.0):
        for n in names:
            if n not in self.subs:
                self.subs[n] = self.create_subscription(
                    DroneState, f'/{n}/state', lambda m, n=n: self.states.__setitem__(n, m), 10)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and any(n not in self.states for n in names):
            rclpy.spin_once(self, timeout_sec=0.1)
        return {n: self.states.get(n) for n in names}

    def call_all(self, srv_type, srv_name, request, names, timeout=30.0):
        futures = {}
        for n in names:
            cli = self.create_client(srv_type, f'/{n}/{srv_name}')
            if not cli.wait_for_service(timeout_sec=5.0):
                print(f'  {n}: service /{n}/{srv_name} not available')
                continue
            futures[n] = cli.call_async(request)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and not all(f.done() for f in futures.values()):
            rclpy.spin_once(self, timeout_sec=0.1)
        ok = True
        for n, f in futures.items():
            r = f.result() if f.done() else None
            print(f'  {n}: {r.message if r else "no response"}')
            ok = ok and r is not None and r.success
        return ok and len(futures) == len(names)

    def send_positions(self, targets, yaw=None):
        """targets: {name: (x, y, z)} in the map frame."""
        pubs = {n: self.create_publisher(PoseStamped, f'/{n}/cmd/position', 10) for n in targets}
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0 and any(p.get_subscription_count() == 0 for p in pubs.values()):
            rclpy.spin_once(self, timeout_sec=0.1)
        for _ in range(3):
            for n, (x, y, z) in targets.items():
                if pubs[n].get_subscription_count() == 0:
                    continue
                msg = PoseStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
                if yaw is None:
                    msg.pose.orientation.w = 0.0     # keep heading
                else:
                    msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                pubs[n].publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        for n, (x, y, z) in targets.items():
            ok = pubs[n].get_subscription_count() > 0
            print(f'  {n}: ({x:7.2f}, {y:7.2f}, {z:6.2f})' + ('' if ok else '  NOT SENT (no subscriber)'))


def print_states(states):
    print(f'{"drone":10s} {"link":5s} {"armed":6s} {"mode":10s} {"x":>8s} {"y":>8s} {"z":>7s} {"yaw":>6s} '
          f'{"|v|":>5s} {"bat":>6s} {"cmd":>6s}')
    for n, s in states.items():
        if s is None:
            print(f'{n:10s} (no state)')
            continue
        v = math.sqrt(s.velocity.x ** 2 + s.velocity.y ** 2 + s.velocity.z ** 2)
        pos = (f'{s.position.x:8.2f} {s.position.y:8.2f} {s.position.z:7.2f} {math.degrees(s.yaw):6.1f}'
               if s.position_valid else f'{"-":>8s} {"-":>8s} {"-":>7s} {"-":>6s}')
        print(f'{n:10s} {"ok" if s.connected else "LOST":5s} {"yes" if s.armed else "no":6s} {s.mode:10s} {pos} '
              f'{v:5.1f} {s.battery_voltage:5.1f}V {SETPOINT_NAMES.get(s.setpoint_type, "?"):>6s}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-d', '--drones', nargs='+', help='ids, ranges (1-4) or names. Default: discover')
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('state')
    p.add_argument('--watch', action='store_true')
    p = sub.add_parser('takeoff')
    p.add_argument('altitude', type=float)
    p.add_argument('--alt-step', type=float, default=0.0, help='extra altitude per drone [m]')
    for name in ('land', 'rtl', 'hold'):
        sub.add_parser(name)
    p = sub.add_parser('formation')
    p.add_argument('shape', choices=['line', 'column', 'circle', 'grid', 'v'])
    p.add_argument('--spacing', type=float, default=4.0)
    p.add_argument('--alt', type=float, default=10.0)
    p.add_argument('--alt-step', type=float, default=0.0)
    p.add_argument('--center', type=float, nargs=2, default=None, help='x y (default: current centroid)')
    p.add_argument('--heading', type=float, default=0.0, help='deg ENU')
    p.add_argument('--face-heading', action='store_true', help='also turn the drones to the heading')
    p = sub.add_parser('goto')
    p.add_argument('x', type=float)
    p.add_argument('y', type=float)
    p.add_argument('z', type=float)
    p.add_argument('--shape', default='line', choices=['line', 'column', 'circle', 'grid', 'v'])
    p.add_argument('--spacing', type=float, default=4.0)
    # "-d 1 2 3 takeoff 10": argparse would take the command as one more drone (nargs='+'),
    # so the drone list is cut at the first command word before parsing
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    drones = []
    if argv and argv[0] in ('-d', '--drones'):
        k = 1
        while k < len(argv) and argv[k] not in COMMANDS:
            drones += [d for d in argv[k].split(',') if d]
            k += 1
        argv = argv[k:]
    args = parser.parse_args(argv)
    args.drones = drones or None

    rclpy.init()
    node = SwarmCli()
    code = 0
    try:
        names = parse_drones(args.drones) or node.discover()
        if not names:
            print('no drones found (no /<name>/state topic). Is the GCS Zenoh bridge running? '
                  'Or use -d and the ROS_DOMAIN_ID of a drone')
            sys.exit(1)
        print(f'drones: {" ".join(names)}')

        if args.cmd == 'state':
            while True:
                print_states(node.collect_states(names))
                if not args.watch:
                    break
                time.sleep(1.0)
                print()
        elif args.cmd == 'takeoff':
            if args.alt_step == 0.0:
                ok = node.call_all(Takeoff, 'takeoff', Takeoff.Request(altitude=args.altitude), names)
            else:
                ok = all(node.call_all(Takeoff, 'takeoff', Takeoff.Request(altitude=args.altitude + k * args.alt_step), [n])
                         for k, n in enumerate(names))
            code = 0 if ok else 1
        elif args.cmd in ('land', 'rtl', 'hold'):
            code = 0 if node.call_all(Trigger, args.cmd, Trigger.Request(), names) else 1
        elif args.cmd in ('formation', 'goto'):
            states = node.collect_states(names)
            valid = [n for n in names if states[n] is not None and states[n].position_valid]
            if len(valid) != len(names):
                print(f'  without position: {[n for n in names if n not in valid]}')
            if not valid:
                sys.exit(1)
            if args.cmd == 'formation':
                shape, spacing, alt, alt_step = args.shape, args.spacing, args.alt, args.alt_step
                if args.center is None:
                    center = (sum(states[n].position.x for n in valid) / len(valid),
                              sum(states[n].position.y for n in valid) / len(valid))
                else:
                    center = tuple(args.center)
                heading = math.radians(args.heading)
            else:
                shape, spacing, alt, alt_step = args.shape, args.spacing, args.z, 0.0
                center, heading = (args.x, args.y), 0.0
            xy = formations.to_map(formations.slots(shape, len(valid), spacing), center, heading)
            # assign slots greedily to the closest drones (avoids crossing paths)
            free = list(range(len(xy)))
            targets = {}
            for n in sorted(valid, key=lambda n: states[n].position.x):
                p = states[n].position
                k = min(free, key=lambda i: (xy[i][0] - p.x) ** 2 + (xy[i][1] - p.y) ** 2)
                free.remove(k)
                targets[n] = (xy[k][0], xy[k][1], alt + valid.index(n) * alt_step)
            node.send_positions(targets, heading if getattr(args, 'face_heading', False) else None)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
