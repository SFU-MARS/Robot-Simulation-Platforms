"""
fastbrt_nav: LiDAR navigation in an unknown map with the FaST-BRT safety filter.

The same loop as ``nav_core.sim.run_episode``, wired to ROS 2 (Humble or Jazzy):

  /scan, /odom --> accumulated occupancy grid --> nominal (pursuit | MPPI)
               --> safety filter (optional: backup | hj | hj_backup) --> /cmd_vel

Every control period (the profile's ctrl_dt) the node integrates every scan
received since the last period, each at the LiDAR's pose at the scan's time
stamp (the odometry history interpolated at that time, moved by the profile's
``sensor.offset``: the Burger's LiDAR is 3.2 cm behind the wheel axle), decides
the turn rates for the period from the newest pose, and publishes
them on a sub-period timer (the filters return zero-order-hold rates for their
sub-steps; the timer picks the rate by the time since the period started, so
the order in which the two timers fire does not matter).  Linear speed is the profile's constant v while driving and 0
before the start, at the goal, and after a filter failure.

Needs ``nav_core`` on PYTHONPATH; the filters also need fast-brt's
``fast_brt.query`` and ``fast_brt.corner`` (numpy only; on PYTHONPATH, or the
fast-brt submodule of the repo) and the profile's V0 file (``base_value``, or
the ``v0`` parameter; ``nav_core/scripts/fetch_v0.py`` downloads it).

Leaking corners (docs/13): ``corners`` "auto" uses the filter's default (on
for hj, off for backup and hj_backup), "true" / "false" override it. Online the
corner windows are solved on a background thread (``corner_mode`` "thread"):
until a window is ready the filter uses the previous one, or the union only.
Odometry starts at the spawn pose: pass the spawn pose as parameters.

Published for RViz (frame ``world_frame``): ``map`` (nav_msgs/OccupancyGrid)
and ``fastbrt_markers`` (obstacle faces, convex / concave corners, filter
region).  ``obstacle_events`` (std_msgs/String, one JSON object per event):
obstacle detection events with stable ids (new / updated / merged / split /
removed), faces with lengths, and corners, in world coordinates.
``log_path``: CSV per period: t, x, y, theta, w_nom, w, decide_s (seconds to
decide), scan_age (seconds since the newest integrated scan), intervened, interventions, corner_windows (in use), corner_solves,
corner_seconds (cumulative solve time). ``fastbrt_markers`` also shows the
usable part of every corrected corner window (green squares).
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

from .conversions import (PoseHistory, odom_to_world, occupancy_data, scan_arrays, segments_points, stamp_seconds,
                          yaw_from_quaternion)


class FastBrtNav(Node):
    def __init__(self):
        super().__init__("fastbrt_nav")
        P = self.declare_parameter
        P("robot", "burger")                      # profile name (robots/<name>.yaml) or path
        P("goal", [0.7, 0.2])                     # world frame, m
        P("goal_radius", 0.1)
        P("spawn", [-0.8, -0.8, 0.2])             # spawn pose (x, y, yaw) = odom origin in the world
        P("nominal", "mppi")                      # pursuit | mppi
        P("filter", "backup")                     # none | backup | hj | hj_backup | hj_cbf
        P("unknown", "occupied")                  # backup filter mode: occupied | free
        P("primitives", "faces")                  # backup filter primitives: faces | rings
        P("v0", "")                               # filter V0 file; "": the profile's base_value
        P("corners", "auto")                      # leaking-corner correction: auto (filter default) | true | false
        P("corner_mode", "thread")                # thread (online) | sync | delayed
        P("margin", -1.0)                         # hj / hj_backup distance margin (m); < 0: the filter's default
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
        if g("filter") != "none":
            from nav_core.safety import make_filter
            kw = dict(unknown=g("unknown"), primitives=g("primitives"), corner_mode=g("corner_mode"))
            corners = str(g("corners")).lower()
            if corners in ("true", "false"):
                kw["corners"] = corners == "true"
            elif corners != "auto":
                raise ValueError(f"corners must be auto, true or false, got {g('corners')!r}")
            if g("filter") in ("hj", "hj_backup", "hj_cbf") and float(g("margin")) >= 0:
                kw["margin"] = float(g("margin"))
            self.safety = make_filter(self.profile, g("filter"), g("v0") or None, **kw)

        self.pose = None                          # world frame (x, y, theta), newest
        self.history = PoseHistory()              # recent poses with their time stamps
        self.scans = []                           # scans not integrated yet: (time stamp, angles, ranges)
        self.scan_time = None                     # time stamp of the newest integrated scan
        self.state = "waiting"                    # waiting | driving | reached | stopped
        self.queue = []                           # turn rates of this period, one per sub-step
        self.period_start = 0.0                   # time (s) the rates of this period start
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
            f"({g('unknown')}, {g('primitives')}, corners "
            f"{getattr(self.safety, 'corners', False)} {g('corner_mode')}); goal {self.goal.tolist()}")

    # -- inputs
    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        self.pose = odom_to_world((p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)), self.spawn)
        self.history.add(stamp_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec), self.pose)

    def on_scan(self, msg: LaserScan):
        angles, ranges = scan_arrays(msg.angle_min, msg.angle_increment, msg.ranges, msg.range_min, msg.range_max,
                                     self.profile.sensor.range)
        self.scans.append((stamp_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec), angles, ranges))
        del self.scans[:-10]                      # at most 10 pending (2 s at 5 Hz)

    # -- control
    def on_control(self):
        if self.state in ("reached", "stopped"):
            return
        if self.pose is None or (self.scan_time is None and not self.scans):
            return
        x = self.pose.copy()
        s = self.profile.sensor
        for stamp, angles, ranges in self.scans:  # each scan at the LiDAR pose at its own time
            pose = self.history.at(stamp) if len(self.history) else x
            self.grid.integrate_scan(self.profile.sensor_pose(pose), angles, ranges, s.range, s.range_min)
            self.scan_time = stamp
        self.scans = []
        self.obstacles, events = self.tracker.update(extract_obstacles(self.grid), self.elapsed(), self.grid.res)
        for e in events:
            self.event_pub.publish(String(data=json.dumps(e.to_dict())))
            self.get_logger().info(e.summary())
        if np.linalg.norm(x[:2] - self.goal) <= self.goal_radius:
            self.finish("reached", f"goal reached at {x[:2].round(3).tolist()}")
            return
        t = time.perf_counter()
        w_nom = self.nominal.control(x, self.goal, self.grid)
        before = getattr(self.safety, "interventions", 0)
        try:
            ws = np.atleast_1d(w_nom if self.safety is None else self.safety(x, w_nom, self.grid))
        except RuntimeError as e:                 # no certificate at the start, unsupported case, filter failure
            self.finish("stopped", f"filter stopped the robot: {e}")
            return
        w_max = self.profile.phys["w_max"]
        self.queue = [float(np.clip(w, -w_max, w_max)) for w in ws]
        self.period_start = self.get_clock().now().nanoseconds * 1e-9
        if self.state == "waiting":
            self.state = "driving"
        self.publish_rate()                       # the first rate at once
        f = self.safety
        model = getattr(f, "model", None)
        now = self.get_clock().now().nanoseconds * 1e-9
        self.log.append((self.elapsed(), *x.tolist(), float(w_nom), self.queue[0], time.perf_counter() - t,
                         round(now - self.scan_time, 3),
                         int(getattr(f, "interventions", 0) > before), getattr(f, "interventions", 0),
                         len(getattr(model, "corners", []) or []), getattr(f, "corner_solves", 0),
                         round(getattr(f, "corner_seconds", 0.0), 3)))
        self.publish_viz()

    def elapsed(self) -> float:
        """Seconds since the first control period with odometry and a scan."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        return now - self.t0

    def on_sub_step(self):
        self.publish_rate()

    def publish_rate(self):
        """The rate of the current sub-step, chosen by the time since the period started (not popped from a queue:
        the two timers can fire in either order at a period boundary); after the last sub-step the last rate is
        held until the next period."""
        cmd = Twist()
        if self.state == "driving" and self.queue:
            now = self.get_clock().now().nanoseconds * 1e-9
            k = int((now - self.period_start) / self.sub_dt + 1e-6)
            cmd.linear.x = float(self.profile.phys["v"])
            cmd.angular.z = self.queue[min(max(k, 0), len(self.queue) - 1)]
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
            w.writerow(["t", "x", "y", "theta", "w_nom", "w", "decide_s", "scan_age", "intervened", "interventions",
                        "corner_windows", "corner_solves", "corner_seconds"])
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
        boxes = []                                # usable part of the corrected corner windows (model -> world)
        model = getattr(self.safety, "model", None)
        for w in getattr(model, "corners", []) or []:
            c = self.profile.units.pos_to_phys(np.asarray(w.grid.center))
            h = float(self.profile.units.pos_to_phys(w.grid.half - w.edge))
            pts = [c + np.array(d) * h for d in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
            boxes += [(pts[k], pts[(k + 1) % 4]) for k in range(4)]
        arr.markers.append(self.lines(4, "corner_windows", boxes, (0.1, 0.7, 0.3), stamp))
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
