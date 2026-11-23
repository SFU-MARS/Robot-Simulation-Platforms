"""
Closed-loop batches of MPPI + backup filter (Burger, unknown occupied) in three worlds, for comparing V0 files
(encoding, scheme accuracy, alpha) and placements.  The worlds, starts, goals and metrics are those of
docs/10 section 10.5 (experiments/alpha_study.py closedloop).

    cd nav_core
    PYTHONPATH=. python experiments/filter_batch.py                                    # the profile's V0
    PYTHONPATH=. python experiments/filter_batch.py --v0 ../fast-brt/results/v0/v0_dubins_3d_linear_alpha20_low.npz \\
        ../fast-brt/results/v0/v0_dubins_3d_quadratic_alpha20_low.npz
    PYTHONPATH=. python experiments/filter_batch.py --placements inward centered+cut --worlds hj

Writes outputs/filter_batch.csv (--out).
"""

from __future__ import annotations

import argparse
import collections
import csv
import time
from pathlib import Path

import numpy as np

from nav_core.mapping import OccupancyGrid
from nav_core.nominal import make_nominal
from nav_core.robots import ROBOTS_DIR, load_profile
from nav_core.safety import BackupFilter, load_v0
from nav_core.sim import Box, World, run_episode

PLACEMENTS = {"inward": dict(placement="inward", cut=False), "centered+cut": dict(placement="centered", cut=True)}


def worlds(profile):
    root = ROBOTS_DIR.parent
    hj = World.from_sdf(root / profile.world, plane_z=profile.sensor.height)
    occ = World.from_sdf(root / "turtlebot3/turtlebot3_controls/worlds/occluded_4x4.world",
                         plane_z=profile.sensor.height)
    walls = [b for b in hj.boxes if max(b.half) > 0.9]
    return {
        "hj": {"w": hj, "xy": (-0.85, 0.85), "goals": [(0.7, 0.2), (0.65, 0.8), (-0.6, 0.6)], "n": 10, "grid": None},
        "pillar": {"w": World(walls + [Box((0.0, 0.0), (0.15, 0.15))]), "xy": (-0.85, 0.85),
                   "goals": [(0.6, 0.6), (-0.6, 0.6), (0.6, -0.6), (-0.6, -0.6)], "n": 8, "grid": None},
        "occluded": {"w": occ, "xy": (-1.8, 1.8), "goals": [(1.7, 0.6), (-1.6, -0.4), (0.9, -1.6), (-0.3, 1.7)],
                     "n": 6, "grid": occ.bounds()},
    }


def cases(world, profile, seed=1):
    """Random starts (away from obstacles) x goals farther than 0.4 m, as in alpha_study."""
    rng = np.random.default_rng(seed)
    starts = []
    while len(starts) < world["n"]:
        x = np.r_[rng.uniform(*world["xy"], 2), rng.uniform(-np.pi, np.pi)]
        if world["w"].distance(x[None, :2])[0] > profile.footprint_radius + 0.12:
            starts.append(x)
    goals = [np.array(g) for g in world["goals"]]
    return [(x0, g) for x0 in starts for g in goals if np.linalg.norm(x0[:2] - g) > 0.4]


def run(profile, world, v0, kw, unknown="occupied", t_max=100.0):
    c, clr, interv, prims, t_goal, steps, t0 = collections.Counter(), np.inf, [], [], [], 0, time.perf_counter()
    todo = cases(world, profile)
    for i, (x0, goal) in enumerate(todo):
        gate = BackupFilter(profile, v0, unknown=unknown, **kw)
        grid = None if world["grid"] is None else OccupancyGrid(*world["grid"], profile.map.resolution)
        ep = run_episode(profile, world["w"], x0, goal, make_nominal(profile, "mppi", seed=i), safety=gate,
                         t_max=t_max, seed=i, grid=grid)
        c[ep.status] += 1
        steps += len(ep.cmds)
        if ep.status == "reached":
            t_goal.append(ep.t)
        if len(ep.traj) > 1:
            clr = min(clr, float(world["w"].distance(ep.traj[:, :2]).min() - profile.footprint_radius))
        if gate.steps:
            interv.append(gate.interventions / gate.steps)
        if gate.model is not None:
            prims.append(len(gate.model.frames))
    return {"episodes": len(todo), "reached": c["reached"], "collision": c["collision"], "timeout": c["timeout"],
            "other": len(todo) - c["reached"] - c["collision"] - c["timeout"],
            "min_clearance_m": round(clr, 4), "intervention_rate": round(float(np.mean(interv)) if interv else 0.0, 3),
            "median_time_to_goal_s": round(float(np.median(t_goal)) if t_goal else 0.0, 1),
            "primitives_last": round(float(np.mean(prims)) if prims else 0.0, 1),
            "ms_per_step": round(1000 * (time.perf_counter() - t0) / max(steps, 1), 1)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--v0", nargs="+", default=None, help="V0 files (default: the profile's base_value)")
    ap.add_argument("--placements", nargs="+", default=["inward"], choices=list(PLACEMENTS))
    ap.add_argument("--worlds", nargs="+", default=["hj", "pillar", "occluded"], choices=["hj", "pillar", "occluded"])
    ap.add_argument("--unknown", default="occupied", choices=["occupied", "free"])
    ap.add_argument("--out", default=str(Path(__file__).resolve().parents[1] / "outputs" / "filter_batch.csv"))
    args = ap.parse_args()

    profile = load_profile("burger")
    all_worlds = worlds(profile)
    rows = []
    for path in args.v0 or [profile.base_value]:
        v0 = load_v0(path)
        name = f"{v0.encoding}, {v0.meta.get('accuracy', '?')}, alpha {v0.alpha:g}"
        for wname in args.worlds:
            for placement in args.placements:
                row = {"world": wname, "v0": name, "placement": placement,
                       **run(profile, all_worlds[wname], v0, PLACEMENTS[placement], args.unknown)}
                rows.append(row)
                print(" | ".join(f"{k} {v}" for k, v in row.items()), flush=True)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
