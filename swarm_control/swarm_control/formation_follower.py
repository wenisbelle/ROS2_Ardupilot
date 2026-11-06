#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
formation_follower: example of a DECENTRALISED swarm behaviour (template for
your own algorithms).

Runs on the follower drone (its namespace /<name> and its ROS domain). It
reads the state of the leader, which lives in ANOTHER domain / computer and
arrives through the Zenoh bridges, and keeps a slot relative to it:

    target = leader_position + R(leader_yaw) * offset      (rotate_with_leader)
    target = leader_position + offset                       (otherwise)

It only commands when both drones are flying (armed, GUIDED, above
min_altitude), so it can be left running: take off the drones, then move the
leader (QGC, drone_cli, teleop...) and the followers keep the formation.

  ros2 launch swarm_control formation_follower.launch.py name:=drone2 leader:=drone1 offset:="[-4.0, 4.0, 0.0]"
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from drone_interfaces.msg import DroneState


class FormationFollower(Node):

    def __init__(self):
        super().__init__('formation_follower')
        self.declare_parameter('leader', 'drone1')
        self.declare_parameter('offset', [-4.0, 4.0, 0.0])     # m: back, left, up (leader frame)
        self.declare_parameter('rotate_with_leader', True)
        self.declare_parameter('match_leader_yaw', True)
        self.declare_parameter('rate', 5.0)                    # Hz
        self.declare_parameter('min_altitude', 2.0)            # m, never command below this height
        self.declare_parameter('leader_timeout', 1.5)          # s, stop following when the leader is silent

        self.leader = self.get_parameter('leader').value.strip('/')
        self.offset = [float(v) for v in self.get_parameter('offset').value]
        self.rotate = bool(self.get_parameter('rotate_with_leader').value)
        self.match_yaw = bool(self.get_parameter('match_leader_yaw').value)
        self.min_alt = float(self.get_parameter('min_altitude').value)
        self.timeout = float(self.get_parameter('leader_timeout').value)

        self.me = None
        self.lead = None
        self.lead_time = None
        self.following = False

        # absolute topic of the leader (other namespace / domain, routed by Zenoh)
        self.create_subscription(DroneState, f'/{self.leader}/state', self.on_leader, 10)
        # own state and command (relative: namespace of this drone)
        self.create_subscription(DroneState, 'state', self.on_me, 10)
        self.pub = self.create_publisher(PoseStamped, 'cmd/position', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.step)
        self.get_logger().info(f'following /{self.leader} with offset {self.offset} '
                               f'({"leader" if self.rotate else "map"} frame)')

    def on_leader(self, msg):
        self.lead = msg
        self.lead_time = self.get_clock().now()

    def on_me(self, msg):
        self.me = msg

    def flying(self, s):
        return s is not None and s.connected and s.armed and s.position_valid and s.position.z > self.min_alt

    def step(self):
        lead_ok = (self.lead_time is not None and
                   (self.get_clock().now() - self.lead_time).nanoseconds * 1e-9 < self.timeout)
        active = lead_ok and self.flying(self.lead) and self.flying(self.me) and self.me.guided
        if active != self.following:
            self.following = active
            self.get_logger().info('following the leader' if active else 'waiting (leader / follower not flying in GUIDED)')
        if not active:
            return

        dx, dy, dz = self.offset
        if self.rotate:
            c, s = math.cos(self.lead.yaw), math.sin(self.lead.yaw)
            dx, dy = dx * c - dy * s, dx * s + dy * c
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.lead.position.x + dx
        msg.pose.position.y = self.lead.position.y + dy
        msg.pose.position.z = max(self.min_alt, self.lead.position.z + dz)
        if self.match_yaw:
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(self.lead.yaw / 2), math.cos(self.lead.yaw / 2)
        else:
            msg.pose.orientation.w = 0.0      # keep own heading
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FormationFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
