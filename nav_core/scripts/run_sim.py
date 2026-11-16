"""
ROS-free closed-loop run of one robot profile in a Gazebo SDF world, with the
nominal controller alone or behind the FaST-BRT witness gatekeeper.  Prints
the outcome and the estimated obstacle sides, and plots map snapshots (with
the free polygon the gatekeeper uses).

    cd nav_core
    PYTHONPATH=. python scripts/run_sim.py                                   # nominal only
    PYTHONPATH=.:<path to FaST-BRT> python scripts/run_sim.py --filter gatekeeper
    PYTHONPATH=. python scripts/run_sim.py --goal 0.7 0.2 --x0 -0.8 -0.8 0.2 --out outputs/sim_burger.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from nav_core.mapping import FREE, OCCUPIED, OccupancyGrid, boundary_edges, free_region
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
    g = OccupancyGrid.from_profile(profile)
    ext = [g.origin[0], g.origin[0] + g.nx * g.res, g.origin[1], g.origin[1] + g.ny * g.res]
    for ax, i in zip(axes[0], idx):
        t, pose, lo = snaps[i]
        g.logodds = lo
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
    ap.add_argument("--world", default=None, help="SDF world (default: the profile's)")
    ap.add_argument("--x0", type=float, nargs=3, default=[-0.8, -0.8, 0.2])
    ap.add_argument("--goal", type=float, nargs=2, default=[0.7, 0.2])
    ap.add_argument("--t-max", type=float, default=30.0)
    ap.add_argument("--noise", type=float, default=0.0, help="range noise std (m)")
    ap.add_argument("--filter", default="none", choices=["none", "gatekeeper"],
                    help="gatekeeper: FaST-BRT witness gatekeeper (fast_brt must be importable)")
    ap.add_argument("--unknown", default="occupied", choices=["occupied", "free"],
                    help="gatekeeper: treat unknown space as occupied (frontier edges are obstacles) or free")
    ap.add_argument("--out", default="outputs/sim_burger.png")
    args = ap.parse_args()

    profile = load_profile(args.robot).require_ready()
    world = World.from_sdf(args.world or REPO / profile.world, plane_z=profile.sensor.height)
    safety, title = None, "nominal only"
    if args.filter == "gatekeeper":
        from nav_core.safety import FastBrtGatekeeper
        safety, title = FastBrtGatekeeper(profile, unknown=args.unknown), f"FaST-BRT gatekeeper, unknown {args.unknown}"
    ep = run_episode(profile, world, args.x0, args.goal, make_nominal(profile), safety=safety,
                     t_max=args.t_max, snapshot_every=1.0, noise_std=args.noise)
    clear = float(world.distance(ep.traj[:, :2]).min() - profile.footprint_radius)
    print(f"{profile.name}, {title}: {ep.status} at t = {ep.t:.2f} s, {len(ep.cmds)} control periods, "
          f"min clearance {clear:.3f} m" + (f", {safety.interventions} interventions" if safety else ""))
    for e in sorted(boundary_edges(ep.grid), key=lambda e: -e.length):
        if e.kind == "obstacle" and e.length >= 0.1:
            print(f"  obstacle side {e.length:.3f} m  {np.round(e.a, 3)} -> {np.round(e.b, 3)}")
    plot(ep, world, profile, args.goal, Path(args.out), title, unknown_free=(args.unknown == "free"))
    print(f"figure: {args.out}")


if __name__ == "__main__":
    main()
