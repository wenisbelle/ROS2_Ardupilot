#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
swarm_monitor: ground station view of the swarm (GCS domain).

Discovers every /<name>/state (drone_interfaces/DroneState) that arrives
through the Zenoh bridges and publishes, in the "map" frame:
  /swarm/markers   visualization_msgs/MarkerArray (body, heading, label, trail, target)
  TF map -> <name>/base_link  (GCS copy, for RViz)
and logs a status table every `table_period` seconds.
"""
import math
import re

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from drone_interfaces.msg import DroneState

COLORS = [(0.9, 0.2, 0.2), (0.2, 0.5, 1.0), (0.2, 0.8, 0.3), (1.0, 0.7, 0.1),
          (0.7, 0.3, 0.9), (0.1, 0.8, 0.8), (1.0, 0.4, 0.7), (0.6, 0.6, 0.6)]


class SwarmMonitor(Node):

    def __init__(self):
        super().__init__('swarm_monitor')
        self.declare_parameter('drones', [''])          # empty: discover automatically
        self.declare_parameter('trail_length', 300)
        self.declare_parameter('table_period', 5.0)
        self.trail_len = int(self.get_parameter('trail_length').value)
        self.states = {}
        self.trails = {}
        self.subs = {}
        self.colors = {}
        self.pub = self.create_publisher(MarkerArray, '/swarm/markers', 10)
        self.tf = TransformBroadcaster(self)
        fixed = [d for d in self.get_parameter('drones').value if d]
        for d in fixed:
            self.add(d.strip('/'))
        if not fixed:
            self.create_timer(2.0, self.discover)
        self.create_timer(0.2, self.publish)
        period = float(self.get_parameter('table_period').value)
        if period > 0:
            self.create_timer(period, self.table)

    def discover(self):
        for topic, types in self.get_topic_names_and_types():
            m = re.fullmatch(r'/([^/]+)/state', topic)
            if m and 'drone_interfaces/msg/DroneState' in types and m.group(1) not in self.subs:
                self.add(m.group(1))

    def add(self, name):
        self.colors[name] = COLORS[len(self.subs) % len(COLORS)]
        self.trails[name] = []
        self.subs[name] = self.create_subscription(DroneState, f'/{name}/state',
                                                   lambda m, n=name: self.on_state(n, m), 10)
        self.get_logger().info(f'monitoring /{name}/state')

    def on_state(self, name, msg):
        self.states[name] = msg
        if msg.position_valid:
            trail = self.trails[name]
            p = (msg.position.x, msg.position.y, msg.position.z)
            if not trail or math.dist(trail[-1], p) > 0.2:
                trail.append(p)
                del trail[:-self.trail_len]

    def publish(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, (name, s) in enumerate(sorted(self.states.items())):
            if not s.position_valid:
                continue
            r, g, b = self.colors[name]
            base = i * 10

            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map'
            t.child_frame_id = f'{name}/base_link'
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
                s.position.x, s.position.y, s.position.z
            t.transform.rotation.z, t.transform.rotation.w = math.sin(s.yaw / 2), math.cos(s.yaw / 2)
            self.tf.sendTransform(t)

            body = self.marker(name, base, Marker.CYLINDER, (r, g, b, 0.9))
            body.pose.position = self.point(s.position.x, s.position.y, s.position.z)
            body.scale.x = body.scale.y = 0.8
            body.scale.z = 0.15
            arr.markers.append(body)

            head = self.marker(name, base + 1, Marker.ARROW, (r, g, b, 1.0))
            head.points = [self.point(s.position.x, s.position.y, s.position.z),
                           self.point(s.position.x + 1.2 * math.cos(s.yaw), s.position.y + 1.2 * math.sin(s.yaw),
                                      s.position.z)]
            head.scale.x, head.scale.y, head.scale.z = 0.1, 0.25, 0.3
            arr.markers.append(head)

            label = self.marker(name, base + 2, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
            label.pose.position = self.point(s.position.x, s.position.y, s.position.z + 1.0)
            label.scale.z = 0.6
            label.text = f'{name} {s.mode}{" ARMED" if s.armed else ""}'
            arr.markers.append(label)

            if len(self.trails[name]) > 1:
                trail = self.marker(name, base + 3, Marker.LINE_STRIP, (r, g, b, 0.6))
                trail.scale.x = 0.08
                trail.points = [self.point(*p) for p in self.trails[name]]
                arr.markers.append(trail)

            target = self.marker(name, base + 4, Marker.SPHERE, (r, g, b, 0.35))
            if s.setpoint_type == DroneState.SETPOINT_POSITION:
                target.pose.position = self.point(s.target.x, s.target.y, s.target.z)
                target.scale.x = target.scale.y = target.scale.z = 0.6
            else:
                target.action = Marker.DELETE
            arr.markers.append(target)
        if arr.markers:
            self.pub.publish(arr)

    def marker(self, ns, mid, mtype, rgba):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    @staticmethod
    def point(x, y, z):
        p = Point()
        p.x, p.y, p.z = float(x), float(y), float(z)
        return p

    def table(self):
        if not self.states:
            self.get_logger().info('no drone state received yet (Zenoh bridges running?)')
            return
        lines = ['']
        for name, s in sorted(self.states.items()):
            pos = (f'({s.position.x:7.1f}, {s.position.y:7.1f}, {s.position.z:5.1f})' if s.position_valid else 'no position')
            lines.append(f'  {name:8s} {"ok" if s.connected else "LOST":4s} {"ARMED" if s.armed else "disarm":6s} '
                         f'{s.mode:9s} {pos}')
        self.get_logger().info('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = SwarmMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
