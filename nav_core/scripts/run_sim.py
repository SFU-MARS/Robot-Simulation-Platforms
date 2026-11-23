"""
ROS-free closed-loop run of one robot profile in a Gazebo SDF world, with the
nominal controller alone or behind the backup filter.  Prints the outcome
and the estimated obstacle sides, and plots map snapshots (with the free
polygon the filter uses).  The backup filter reads the profile's V0
(``base_value``) with fast-brt's ``fast_brt.query`` (the fast-brt submodule is
found without PYTHONPATH).

    cd nav_core
    PYTHONPATH=. python scripts/run_sim.py                                   # nominal only
    PYTHONPATH=. python scripts/run_sim.py --filter backup
    PYTHONPATH=. python scripts/run_sim.py --goal 0.7 0.2 --x0 -0.8 -0.8 0.2 --out outputs/sim_burger.png
    PYTHONPATH=. python scripts/run_sim.py --nominal mppi --filter backup \
        --world ../turtlebot3/turtlebot3_controls/worlds/occluded_4x4.world --x0 -0.9 -1.4 0 --goal 0.9 -0.3 --t-max 100
"""

from __future__ import annotations

import argparse
import dataclasses
from pathlib import Path

import numpy as np

from nav_core.mapping import FREE, OCCUPIED, ObstacleTracker, OccupancyGrid, boundary_edges, extract_obstacles, free_region
from nav_core.nominal import make_nominal
from nav_core.robots import ROBOTS_DIR, load_profile
from nav_core.sim import World, run_episode

REPO = ROBOTS_DIR.parent


def plot(ep, world, profile, goal, path, title, unknown_free=False, n_panels=4):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon

    snaps = ep.snapshots
    idx = np.unique(np.linspace(0, len(snaps) - 1, n_panels).round().astype(int))
    fig, axes = plt.subplots(1, len(idx), figsize=(4.2 * len(idx), 4.4), squeeze=False)
    G = ep.grid
    ext = [G.origin[0], G.origin[0] + G.nx * G.res, G.origin[1], G.origin[1] + G.ny * G.res]
    for ax, i in zip(axes[0], idx):
        t, pose, g = snaps[i]                     # the map as it was at time t
        st = g.state()
        img = np.full(st.shape, 0.75)
        img[st == FREE], img[st == OCCUPIED] = 1.0, 0.0
        ax.imshow(img, origin="lower", extent=ext, cmap="gray", vmin=0, vmax=1, interpolation="nearest")
        for b in world.boxes:
            c, s = np.cos(b.yaw), np.sin(b.yaw)
            loc = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * b.half
            ax.add_patch(Polygon(loc @ np.array([[c, s], [-s, c]]) + b.center, fill=False, ec="tab:red", lw=0.8, ls="--"))
        for e in boundary_edges(g):
            ax.plot(*np.stack([e.a, e.b]).T, color="tab:blue" if e.kind == "obstacle" else "tab:orange", lw=1.6)
        outer, holes = free_region(g, pose[:2], footprint_radius=profile.footprint_radius, unknown_free=unknown_free)
        for ring in ([outer] + holes) if outer is not None else []:
            ax.add_patch(Polygon(ring, fill=False, ec="tab:purple", lw=0.8))
        upto = int(round(t / ep.t * (len(ep.traj) - 1))) if ep.t > 0 else 0
        ax.plot(ep.traj[:upto + 1, 0], ep.traj[:upto + 1, 1], color="k", lw=1.0)
        ax.add_patch(Circle(pose[:2], profile.footprint_radius, fill=False, ec="k"))
        ax.add_patch(Circle(pose[:2], profile.sensor.range, fill=False, ec="tab:green", lw=0.6, ls=":"))
        ax.plot(*goal, marker="*", color="tab:green", ms=12)
        ax.set_title(f"t = {t:.1f} s")
        ax.set_xlim(ext[:2]); ax.set_ylim(ext[2:]); ax.set_aspect("equal")
    axes[0][0].plot([], [], color="tab:blue", label="obstacle edge")
    axes[0][0].plot([], [], color="tab:orange", label="frontier edge")
    axes[0][0].plot([], [], color="tab:red", ls="--", label="true obstacle")
    axes[0][0].plot([], [], color="tab:purple", lw=0.8, label="filter region (footprint)")
    axes[0][0].legend(loc="upper left", fontsize=7)
    fig.suptitle(f"{profile.name}, {title}: {ep.status} at t = {ep.t:.1f} s")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=130)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", default="burger")
    ap.add_argument("--world", default=None,
                    help="SDF world (default: the profile's; another world gets a map around its obstacles)")
    ap.add_argument("--x0", type=float, nargs=3, default=[-0.8, -0.8, 0.2])
    ap.add_argument("--goal", type=float, nargs=2, default=[0.7, 0.2])
    ap.add_argument("--t-max", type=float, default=30.0)
    ap.add_argument("--noise", type=float, default=0.0, help="range noise std (m)")
    ap.add_argument("--nominal", default="pursuit", choices=["pursuit", "mppi"])
    ap.add_argument("--sensing-range", type=float, default=None, help="override the profile's LiDAR range (m)")
    ap.add_argument("--filter", default="none", choices=["none", "backup"],
                    help="backup: the backup filter (needs the profile's V0 file)")
    ap.add_argument("--unknown", default="occupied", choices=["occupied", "free"],
                    help="backup filter: treat unknown space as occupied (frontier edges are obstacles) or free")
    ap.add_argument("--v0", default=None, help="backup filter: V0 file (default: the profile's base_value)")
    ap.add_argument("--events", action="store_true", help="print the obstacle detection events")
    ap.add_argument("--out", default="outputs/sim_burger.png")
    args = ap.parse_args()

    profile = load_profile(args.robot).require_ready()
    if args.sensing_range is not None:
        profile = dataclasses.replace(profile, sensor=dataclasses.replace(profile.sensor, range=args.sensing_range))
    world = World.from_sdf(args.world or REPO / profile.world, plane_z=profile.sensor.height)
    grid = None
    if args.world is not None:
        origin, size = world.bounds()
        grid = OccupancyGrid(origin, size, profile.map.resolution)
    safety, title = None, f"{args.nominal} only"
    if args.filter == "backup":
        from nav_core.safety import BackupFilter
        safety = BackupFilter(profile, args.v0, unknown=args.unknown)
        title = f"{args.nominal} + backup filter, unknown {args.unknown}"
    tracker = ObstacleTracker() if args.events else None
    ep = run_episode(profile, world, args.x0, args.goal, make_nominal(profile, args.nominal), safety=safety,
                     t_max=args.t_max, snapshot_every=1.0, noise_std=args.noise, grid=grid, tracker=tracker)
    clear = float(world.distance(ep.traj[:, :2]).min() - profile.footprint_radius)
    print(f"{profile.name}, {title}: {ep.status} at t = {ep.t:.2f} s, {len(ep.cmds)} control periods, "
          f"min clearance {clear:.3f} m" + (f", {safety.interventions} interventions" if safety else ""))
    for e in ep.events:
        print("  " + e.summary())
    om = extract_obstacles(ep.grid)
    if tracker is not None:                       # final summary with the tracked (stable) ids
        om, _ = tracker.update(om, ep.t, ep.grid.res)
    print(f"  {om.n} obstacles, {len(om.faces)} faces, {sum(c.convex for c in om.corners)} convex corners")
    for f in sorted(om.faces, key=lambda f: -f.length):
        if f.length >= 0.1:
            print(f"  obstacle {f.obstacle} face {f.length:.3f} m  {np.round(f.a, 3)} -> {np.round(f.b, 3)}")
    plot(ep, world, profile, args.goal, Path(args.out), title, unknown_free=(args.unknown == "free"))
    print(f"figure: {args.out}")


if __name__ == "__main__":
    main()
