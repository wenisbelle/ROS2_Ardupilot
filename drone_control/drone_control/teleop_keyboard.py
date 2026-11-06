#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Keyboard teleoperation of one drone (velocity in the body frame, GUIDED mode).

  ros2 run drone_control teleop_keyboard drone1

   w / s : forward / backward        r / f : up / down
   a / d : left / right              q / e : yaw left / right
   space : stop (zero velocity)      + / - : speed x1.25 / x0.8
   t     : take off (5 m)            l     : land        h : RTL
   ctrl-c: quit (the drone holds position after the velocity timeout)

Each key press sets the velocity on that axis; it is kept until another key
of the same axis, space, or the key release timeout (0.6 s without keys).
"""
import math
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_srvs.srv import Trigger

from drone_interfaces.srv import Takeoff

KEYS = {  # key: (axis, sign)
    'w': ('x', 1), 's': ('x', -1), 'a': ('y', 1), 'd': ('y', -1),
    'r': ('z', 1), 'f': ('z', -1), 'q': ('yaw', 1), 'e': ('yaw', -1),
}


class Teleop(Node):

    def __init__(self, drone):
        super().__init__(f'teleop_{drone}')
        self.drone = drone
        self.pub = self.create_publisher(TwistStamped, f'/{drone}/cmd/velocity', 10)
        self.cli_takeoff = self.create_client(Takeoff, f'/{drone}/takeoff')
        self.cli_land = self.create_client(Trigger, f'/{drone}/land')
        self.cli_rtl = self.create_client(Trigger, f'/{drone}/rtl')
        self.speed = 2.0
        self.yaw_rate = math.radians(45)
        self.cmd = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.last_key = 0.0
        self.active = False
        self.lock = threading.Lock()
        self.create_timer(0.1, self.tick)

    def key(self, k):
        with self.lock:
            if k in KEYS:
                axis, sign = KEYS[k]
                self.cmd[axis] = sign * (self.yaw_rate if axis == 'yaw' else (self.speed * (0.5 if axis == 'z' else 1.0)))
                self.last_key = time.monotonic()
                self.active = True
            elif k == ' ':
                self.cmd = dict.fromkeys(self.cmd, 0.0)
                self.last_key = time.monotonic()
                self.active = True
            elif k in '+=':
                self.speed = min(10.0, self.speed * 1.25)
            elif k == '-':
                self.speed = max(0.2, self.speed * 0.8)
            elif k == 't':
                self.cli_takeoff.call_async(Takeoff.Request(altitude=5.0))
            elif k == 'l':
                self.active = False
                self.cli_land.call_async(Trigger.Request())
            elif k == 'h':
                self.active = False
                self.cli_rtl.call_async(Trigger.Request())
        self.status()

    def status(self):
        c = self.cmd
        sys.stdout.write(f'\r[{self.drone}] speed {self.speed:4.1f} m/s | vx {c["x"]:5.2f} vy {c["y"]:5.2f} '
                         f'vz {c["z"]:5.2f} yaw {math.degrees(c["yaw"]):6.1f} deg/s      ')
        sys.stdout.flush()

    def tick(self):
        with self.lock:
            if not self.active:
                return
            if time.monotonic() - self.last_key > 0.6:
                self.cmd = dict.fromkeys(self.cmd, 0.0)
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = self.cmd['x'], self.cmd['y'], self.cmd['z']
            msg.twist.angular.z = self.cmd['yaw']
        self.pub.publish(msg)


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    drone = argv[1] if len(argv) > 1 else 'drone1'
    rclpy.init()
    node = Teleop(drone.strip('/'))
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    print(__doc__)
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                node.key(sys.stdin.read(1))
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
