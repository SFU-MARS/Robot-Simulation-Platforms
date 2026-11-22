"""
fastbrt_nav: LiDAR navigation in an unknown map with the FaST-BRT safety filter.

The same loop as ``nav_core.sim.run_episode``, wired to ROS 2 (Humble or Jazzy):

  /scan, /odom --> accumulated occupancy grid --> nominal (pursuit | MPPI)
               --> FaST-BRT witness gatekeeper (optional) --> /cmd_vel

Every control period (the profile's ctrl_dt) the node integrates the latest
scan at the latest pose, decides the turn rates for the period, and publishes
them on a sub-period timer (the gatekeeper returns zero-order-hold rates for
its sub-steps).  Linear speed is the profile's constant v while driving and 0
before the start, at the goal, and after a filter failure.

Needs ``nav_core`` (and, for the gatekeeper, ``fast_brt``) on PYTHONPATH.
Odometry starts at the spawn pose: pass the spawn pose as parameters.

Published for RViz (frame ``world_frame``): ``map`` (nav_msgs/OccupancyGrid)
and ``fastbrt_markers`` (obstacle faces, convex / concave corners, filter
region).  ``obstacle_events`` (std_msgs/String, one JSON object per event):
obstacle detection events with stable ids (new / updated / merged / split /
removed), faces with lengths, and corners, in world coordinates.
``log_path``: CSV of t, x, y, theta, w_nom, w per period.
"""

from __future__ import annotations

import csv
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from nav_core.mapping import FREE, OCCUPIED, UNKNOWN, ObstacleTracker, OccupancyGrid, extract_obstacles, free_region
from nav_core.nominal import make_nominal
from nav_core.robots import load_profile

from .conversions import odom_to_world, occupancy_data, scan_arrays, segments_points, yaw_from_quaternion


class FastBrtNav(Node):
    def __init__(self):
        super().__init__("fastbrt_nav")
        P = self.declare_parameter
        P("robot", "burger")                      # profile name (robots/<name>.yaml) or path
        P("goal", [0.7, 0.2])                     # world frame, m
        P("goal_radius", 0.1)
        P("spawn", [-0.8, -0.8, 0.2])             # spawn pose (x, y, yaw) = odom origin in the world
        P("nominal", "mppi")                      # pursuit | mppi
        P("filter", "gatekeeper")                 # none | gatekeeper
        P("unknown", "occupied")                  # gatekeeper mode: occupied | free
        P("primitives", "faces")                  # gatekeeper primitives: faces | rings
        P("map_origin", [0.0, 0.0])               # with map_size: override the profile's map
        P("map_size", [0.0, 0.0])
        P("world_frame", "odom")                  # frame of the published map / markers (the odom frame)
        P("log_path", "")
        g = lambda name: self.get_parameter(name).value

        self.profile = load_profile(g("robot")).require_ready()
        self.goal = np.array(g("goal"), float)
        self.goal_radius = float(g("goal_radius"))
        self.spawn = np.array(g("spawn"), float)
        self.frame = g("world_frame")
        if any(v > 0 for v in g("map_size")):
            self.grid = OccupancyGrid(g("map_origin"), g("map_size"), self.profile.map.resolution)
        else:
            self.grid = OccupancyGrid.from_profile(self.profile)
        self.nominal = make_nominal(self.profile, g("nominal"))
        self.safety = None
        if g("filter") == "gatekeeper":
            from nav_core.safety import FastBrtGatekeeper
            self.safety = FastBrtGatekeeper(self.profile, unknown=g("unknown"), primitives=g("primitives"))
        elif g("filter") != "none":
            raise ValueError(f"filter must be 'none' or 'gatekeeper', got {g('filter')!r}")

        self.pose = None                          # world frame (x, y, theta)
        self.scan = None                          # (angles, ranges) of the newest scan
        self.state = "waiting"                    # waiting | driving | reached | stopped
        self.queue = []                           # turn rates left for this period
        self.log = []
        self.log_path = g("log_path")
        self.t0 = None
        self.tracker = ObstacleTracker()          # physical obstacle layer (no footprint offset)
        self.obstacles = None                     # its ObstacleMap of the last period (stable ids)

        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.map_pub = self.create_publisher(OccupancyGridMsg, "map", 1)
        self.marker_pub = self.create_publisher(MarkerArray, "fastbrt_markers", 1)
        self.event_pub = self.create_publisher(String, "obstacle_events", 10)
        dt = self.profile.ctrl_dt
        n_sub = self.safety.n_sub if self.safety is not None else 1
        self.sub_dt = dt / n_sub
        self.create_timer(dt, self.on_control)
        self.create_timer(self.sub_dt, self.on_sub_step)
        self.get_logger().info(
            f"robot {self.profile.name}: v {self.profile.phys['v']} m/s, w_max {self.profile.phys['w_max']} rad/s, "
            f"sensing {self.profile.sensor.range} m, ctrl {dt} s; nominal {g('nominal')}, filter {g('filter')} "
            f"({g('unknown')}, {g('primitives')}); goal {self.goal.tolist()}")

    # -- inputs
    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        self.pose = odom_to_world((p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)), self.spawn)

    def on_scan(self, msg: LaserScan):
        self.scan = scan_arrays(msg.angle_min, msg.angle_increment, msg.ranges, msg.range_min, msg.range_max,
                                self.profile.sensor.range)

    # -- control
    def on_control(self):
        if self.state in ("reached", "stopped"):
            return
        if self.pose is None or self.scan is None:
            return
        x = self.pose.copy()
        s = self.profile.sensor
        angles, ranges = self.scan
        self.grid.integrate_scan(x, angles, ranges, s.range, s.range_min)
        self.obstacles, events = self.tracker.update(extract_obstacles(self.grid), self.elapsed(), self.grid.res)
        for e in events:
            self.event_pub.publish(String(data=json.dumps(e.to_dict())))
            self.get_logger().info(e.summary())
        if np.linalg.norm(x[:2] - self.goal) <= self.goal_radius:
            self.finish("reached", f"goal reached at {x[:2].round(3).tolist()}")
            return
        t = time.perf_counter()
        w_nom = self.nominal.control(x, self.goal, self.grid)
        try:
            ws = np.atleast_1d(w_nom if self.safety is None else self.safety(x, w_nom, self.grid))
        except RuntimeError as e:                 # no certificate at the start, unsupported case, filter failure
            self.finish("stopped", f"filter stopped the robot: {e}")
            return
        w_max = self.profile.phys["w_max"]
        self.queue = [float(np.clip(w, -w_max, w_max)) for w in ws]
        if self.state == "waiting":
            self.state = "driving"
        self.log.append((self.elapsed(), *x.tolist(), float(w_nom), self.queue[0], time.perf_counter() - t))
        self.publish_viz()

    def elapsed(self) -> float:
        """Seconds since the first control period with odometry and a scan."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        return now - self.t0

    def on_sub_step(self):
        cmd = Twist()
        if self.state == "driving" and self.queue:
            cmd.linear.x = float(self.profile.phys["v"])
            cmd.angular.z = self.queue.pop(0)
        self.cmd_pub.publish(cmd)

    def finish(self, state: str, message: str):
        self.state = state
        self.queue = []
        self.cmd_pub.publish(Twist())
        (self.get_logger().info if state == "reached" else self.get_logger().error)(message)
        self.write_log()

    def write_log(self):
        if not self.log_path or not self.log:
            return
        with open(self.log_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "theta", "w_nom", "w", "decide_s"])
            w.writerows(self.log)
        self.get_logger().info(f"wrote {self.log_path}")

    # -- visualization
    def publish_viz(self):
        stamp = self.get_clock().now().to_msg()
        g = self.grid
        m = OccupancyGridMsg()
        m.header.frame_id, m.header.stamp = self.frame, stamp
        m.info.resolution = float(g.res)
        m.info.width, m.info.height = g.nx, g.ny
        o = self.world_to_frame(g.origin)
        m.info.origin.position.x, m.info.origin.position.y = float(o[0]), float(o[1])
        yaw = -self.spawn[2]
        m.info.origin.orientation.z, m.info.origin.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
        m.data = occupancy_data(g.state(), UNKNOWN, FREE, OCCUPIED)
        self.map_pub.publish(m)

        r = self.profile.footprint_radius
        om = self.obstacles                       # physical obstacle layer, stable ids
        outer, holes = free_region(g, self.pose[:2], footprint_radius=r)
        rings = [] if outer is None else [outer] + holes
        arr = MarkerArray()
        arr.markers.append(self.lines(0, "faces", [(f.a, f.b) for f in om.faces], (0.1, 0.4, 0.9), stamp))
        arr.markers.append(self.lines(1, "region", [(a, b) for ring in rings
                                                     for a, b in zip(ring, np.roll(ring, -1, axis=0))],
                                      (0.6, 0.2, 0.8), stamp))
        arr.markers.append(self.points(2, "convex_corners", [c.point for c in om.corners if c.convex],
                                       (0.9, 0.1, 0.1), stamp))
        arr.markers.append(self.points(3, "concave_corners", [c.point for c in om.corners if not c.convex],
                                       (0.9, 0.6, 0.1), stamp))
        self.marker_pub.publish(arr)

    def world_to_frame(self, xy):
        """World point -> odom frame (inverse of odom_to_world)."""
        c, s = np.cos(self.spawn[2]), np.sin(self.spawn[2])
        d = np.asarray(xy, float) - self.spawn[:2]
        return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1]])

    def _marker(self, mid, ns, mtype, color, stamp, scale):
        mk = Marker()
        mk.header.frame_id, mk.header.stamp = self.frame, stamp
        mk.ns, mk.id, mk.type, mk.action = ns, mid, mtype, Marker.ADD
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = scale
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = (*color, 1.0)
        return mk

    def lines(self, mid, ns, segments, color, stamp):
        mk = self._marker(mid, ns, Marker.LINE_LIST, color, stamp, 0.01)
        for px, py in segments_points(segments):
            q = self.world_to_frame((px, py))
            mk.points.append(Point(x=float(q[0]), y=float(q[1]), z=0.02))
        return mk

    def points(self, mid, ns, pts, color, stamp):
        mk = self._marker(mid, ns, Marker.SPHERE_LIST, color, stamp, 0.04)
        for p in pts:
            q = self.world_to_frame(p)
            mk.points.append(Point(x=float(q[0]), y=float(q[1]), z=0.03))
        return mk


def main():
    rclpy.init()
    node = FastBrtNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.write_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
