"""
The node end to end without ROS: rclpy and the message types are replaced by stubs, nav_core's simulated world plays
Gazebo (a LiDAR scan by ray casting, odometry from the true pose in the odom frame), and the published cmd_vel is
executed with exact arcs. Checks the parameters, the control callback, the CSV log and the markers before the first
run in Gazebo. Needs nav_core on PYTHONPATH and the profile's V0 (nav_core/scripts/fetch_v0.py); skipped otherwise.

    cd ros2/fastbrt_nav && PYTHONPATH=.:../../nav_core python -m pytest -q test/test_node_without_ros.py -s
"""

import csv
import math
import sys
import types

import numpy as np
import pytest

nav_core = pytest.importorskip("nav_core")


class Msg:
    """Any message: fields appear on first use (nested), like the fields of a ROS message."""

    def __init__(self, **kw):
        self.__dict__.update(kw)

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        child = Msg()
        setattr(self, name, child)
        return child


class Pub:
    def __init__(self, topic, sink):
        self.topic, self.sink = topic, sink

    def publish(self, msg):
        self.sink.setdefault(self.topic, []).append(msg)


class StubNode:
    """The parts of rclpy.node.Node the node uses; parameters can be preset in ``preset``."""
    preset = {}

    def __init__(self, name):
        self._params = dict()
        self.published = {}
        self.sim_t = 0.0

    def declare_parameter(self, key, default):
        self._params[key] = self.preset.get(key, default)

    def get_parameter(self, key):
        return Msg(value=self._params[key])

    def create_subscription(self, *a, **k):
        pass

    def create_publisher(self, typ, topic, depth):
        return Pub(topic, self.published)

    def create_timer(self, *a):
        pass

    def get_logger(self):
        return Msg(info=lambda s: None, error=lambda s: print("[error]", s))

    def get_clock(self):
        return Msg(now=lambda: Msg(nanoseconds=int(self.sim_t * 1e9), to_msg=lambda: Msg()))

    def destroy_node(self):
        pass


def install_stubs():
    def msg_module(name, classes):
        module = types.ModuleType(name)
        for c in classes:
            setattr(module, c, type(c, (Msg,), {"ADD": 0, "LINE_LIST": 5, "SPHERE_LIST": 7}))
        return module
    rclpy = types.ModuleType("rclpy")
    rclpy.init = rclpy.shutdown = lambda *a: None
    rclpy.ok = lambda: True
    node = types.ModuleType("rclpy.node")
    node.Node = StubNode
    qos = types.ModuleType("rclpy.qos")
    qos.qos_profile_sensor_data = None
    modules = {"rclpy": rclpy, "rclpy.node": node, "rclpy.qos": qos}
    for package, classes in (("geometry_msgs", ["Point", "Twist"]), ("nav_msgs", ["OccupancyGrid", "Odometry"]),
                             ("sensor_msgs", ["LaserScan"]), ("std_msgs", ["String"]),
                             ("visualization_msgs", ["Marker", "MarkerArray"])):
        modules[package] = types.ModuleType(package)
        modules[package + ".msg"] = msg_module(package + ".msg", classes)
    markers = modules["visualization_msgs.msg"]
    markers.MarkerArray.__init__ = lambda self, **kw: self.__dict__.update(kw, markers=[])
    markers.Marker.__init__ = lambda self, **kw: self.__dict__.update(kw, points=[])
    return modules


@pytest.fixture
def node_class(monkeypatch):
    for name, module in install_stubs().items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.delitem(sys.modules, "fastbrt_nav.nav_node", raising=False)
    from fastbrt_nav.nav_node import FastBrtNav
    from nav_core.robots import load_profile
    from nav_core.safety.scene import REPO_ROOT
    if not (REPO_ROOT / load_profile("burger").base_value).is_file():
        pytest.skip("no V0 file (python scripts/fetch_v0.py in nav_core)")
    return FastBrtNav


def stamp(t):
    return Msg(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))


def drive(FastBrtNav, params, x0=(-0.8, -0.8, 0.2), goal=(0.7, 0.2), t_max=40.0, scan_phase=1):
    """Run the node in the simulated world, like Gazebo: odometry every 0.05 s (one sub-step), a scan once per
    control period at sub-step ``scan_phase`` from the LiDAR's pose then (so it is used later, at the next control
    period, while the robot has turned on); both time-stamped. Returns (node, min clearance in m, scan error in m:
    the largest misplacement of a return at the sensing range, from the pose the node integrated each scan at)."""
    from nav_core.robots import ROBOTS_DIR
    from nav_core.sim import World
    from nav_core.sim.loop import unicycle_step
    StubNode.preset = dict(params, spawn=list(x0), goal=list(goal))
    node = FastBrtNav()
    p = node.profile
    world = World.from_sdf(ROBOTS_DIR.parent / p.world, plane_z=p.sensor.height)
    s = p.sensor
    angles = np.arange(s.n_rays) * s.fov / s.n_rays
    x, worst = np.array(x0, float), np.inf
    c, sn = math.cos(x0[2]), math.sin(x0[2])
    n_sub = 4
    h = p.ctrl_dt / n_sub

    def publish(t):
        d = x[:2] - np.array(x0[:2])
        yaw = x[2] - x0[2]                                         # odometry starts at the spawn pose
        node.on_odom(Msg(header=Msg(stamp=stamp(t)),
                         pose=Msg(pose=Msg(position=Msg(x=c * d[0] + sn * d[1], y=-sn * d[0] + c * d[1]),
                                           orientation=Msg(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))))

    def scan(t):
        xs = p.sensor_pose(x)
        ranges = world.raycast(xs[:2], xs[2] + angles, s.range)
        node.on_scan(Msg(header=Msg(stamp=stamp(t)), angle_min=0.0, angle_increment=s.fov / s.n_rays,
                         ranges=list(ranges), range_min=s.range_min, range_max=s.range))

    used = []                                                      # (pose the node used, true LiDAR pose)
    truth = {}
    integrate = node.grid.integrate_scan

    def record(pose, *args, **kw):
        used.append((np.asarray(pose, float).copy(), truth[round(node.scan_time_next, 6)]))
        return integrate(pose, *args, **kw)

    node.grid.integrate_scan = record
    pending = []

    def scan_now(t):
        truth[round(t, 6)] = p.sensor_pose(x)
        pending.append(t)
        scan(t)

    publish(0.0)
    scan_now(0.0)
    while node.sim_t < t_max and node.state not in ("reached", "stopped"):
        stamps = list(pending)
        pending.clear()
        node.scan_time_next = stamps[0] if stamps else None
        if stamps:                                                 # the node integrates pending scans in order
            original = record

            def ordered(pose, *a, _it=iter(stamps), **k):
                node.scan_time_next = next(_it)
                return original(pose, *a, **k)
            node.grid.integrate_scan = ordered
        node.on_control()                                          # publishes the first rate of the period
        node.grid.integrate_scan = record
        for k in range(n_sub):
            if k:
                node.on_sub_step()
            cmd = node.published["cmd_vel"][-1]
            v = cmd.linear.__dict__.get("x", 0.0)
            w = cmd.angular.__dict__.get("z", 0.0)
            x = unicycle_step(x, v if isinstance(v, float) else 0.0, w if isinstance(w, float) else 0.0, h)
            worst = min(worst, world.distance(x[None, :2])[0] - p.footprint_radius)
            node.sim_t += h
            publish(node.sim_t)
            if k == scan_phase:
                scan_now(node.sim_t)
    node.write_log()
    # how far a return at the sensing range lands from where it should: position error + range x heading error
    error = max((float(np.hypot(*(u[:2] - t[:2]))) + s.range * abs((u[2] - t[2] + np.pi) % (2 * np.pi) - np.pi)
                 for u, t in used), default=0.0)
    return node, worst, error


@pytest.mark.parametrize("params", [dict(filter="backup"), dict(filter="hj", nominal="pursuit"),
                                    dict(filter="hj_backup", margin=0.03), dict(filter="none")])
def test_node_reaches_the_goal(node_class, params, tmp_path):
    log = tmp_path / "log.csv"
    node, worst, error = drive(node_class, dict(params, log_path=str(log)))
    assert node.state == "reached", node.state
    assert worst > 0.0                                             # no collision on the way
    assert node.published["cmd_vel"] and node.published["map"]
    namespaces = {m.ns for arr in node.published["fastbrt_markers"] for m in arr.markers}
    assert {"faces", "region", "corner_windows"} <= namespaces
    rows = list(csv.DictReader(open(log)))
    assert rows and {"decide_s", "scan_age", "intervened", "interventions", "corner_windows",
                     "corner_solves"} <= set(rows[0])
    assert error < 0.005                                           # every scan integrated at its true LiDAR pose
    if params["filter"] == "hj":                                   # hj corrects the corners by default
        assert node.safety.corners and node.safety.corner_mode == "thread"
        assert int(rows[-1]["interventions"]) > 0


def test_corner_and_margin_parameters(node_class):
    StubNode.preset = dict(filter="hj_backup", corners="true", corner_mode="sync", margin=0.05)
    node = node_class()
    assert node.safety.corners and node.safety.corner_mode == "sync"
    assert node.safety.margin == pytest.approx(0.05)
    StubNode.preset = dict(filter="hj", corners="false")
    assert not node_class().safety.corners
    StubNode.preset = dict(filter="hj", corners="maybe")
    with pytest.raises(ValueError, match="corners"):
        node_class()


def test_scans_are_integrated_at_their_own_time(node_class, monkeypatch):
    """A scan is used up to a control period after it was taken, while the robot turns at up to 1 rad/s: integrated
    at the newest pose instead of the pose at its time stamp, the obstacles are mapped several cm off"""
    from fastbrt_nav import conversions
    params = dict(filter="none", nominal="pursuit")
    node, _, aligned = drive(node_class, params, t_max=8.0, scan_phase=0)
    monkeypatch.setattr(conversions.PoseHistory, "at", lambda self, t: self.poses[-1].copy())
    _, _, newest = drive(node_class, params, t_max=8.0, scan_phase=0)
    assert aligned < 0.005 and newest > 0.03                       # several cm at 0.8 m when not aligned
