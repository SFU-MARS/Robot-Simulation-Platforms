"""
Closed-loop batches of the Burger in three worlds, for comparing nominal controllers, the filters (backup, hj, hj_backup)
and their modes, V0 files (encoding, scheme accuracy, alpha), placements and LiDAR ranges.  The worlds, starts,
goals and metrics are those of docs/10 section 10.5 (experiments/alpha_study.py closedloop); every combination
of the options below is run.

    cd nav_core
    PYTHONPATH=. python experiments/filter_batch.py                                    # MPPI + backup, the profile's V0
    PYTHONPATH=. python experiments/filter_batch.py --v0 ../fast-brt/results/v0/v0_dubins_3d_linear_alpha20_low.npz \\
        ../fast-brt/results/v0/v0_dubins_3d_quadratic_alpha20_low.npz
    PYTHONPATH=. python experiments/filter_batch.py --placements inward centered+cut --worlds hj
    PYTHONPATH=. python experiments/filter_batch.py --nominals pursuit mppi --filters none backup --unknowns occupied free
    PYTHONPATH=. python experiments/filter_batch.py --filters backup hj hj_backup
    PYTHONPATH=. python experiments/filter_batch.py --filters hj hj_backup --worlds hj occluded --corners --corner-mode delayed
    PYTHONPATH=. python experiments/filter_batch.py --worlds occluded --ranges 0.3 0.5 0.8 --filters none backup \\
        --unknowns occupied free

Writes outputs/filter_batch.csv (--out).
"""

from __future__ import annotations

import argparse
import collections
import csv
import dataclasses
import time
from pathlib import Path

import numpy as np

from nav_core.mapping import OccupancyGrid
from nav_core.nominal import make_nominal
from nav_core.robots import ROBOTS_DIR, load_profile
from nav_core.safety import load_v0, make_filter
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


def run(profile, world, v0, kw, unknown="occupied", nominal="mppi", kind="backup", t_max=100.0, corner_kw=None):
    """One batch with filter ``kind`` (backup | hj | hj_backup | none); v0 and kw (placement) are ignored without a filter."""
    c, clr, interv, prims, t_goal, steps, t0 = collections.Counter(), np.inf, [], [], [], 0, time.perf_counter()
    solves, corner_s = 0, 0.0
    todo = cases(world, profile)
    for i, (x0, goal) in enumerate(todo):
        gate = make_filter(profile, kind, v0, unknown=unknown, **(dict(kw, **(corner_kw or {})) if kind != "none" else {}))
        grid = None if world["grid"] is None else OccupancyGrid(*world["grid"], profile.map.resolution)
        nom = make_nominal(profile, nominal, **({"seed": i} if nominal == "mppi" else {}))
        ep = run_episode(profile, world["w"], x0, goal, nom, safety=gate, t_max=t_max, seed=i, grid=grid)
        c[ep.status] += 1
        steps += len(ep.cmds)
        if ep.status == "reached":
            t_goal.append(ep.t)
        if len(ep.traj) > 1:
            clr = min(clr, float(world["w"].distance(ep.traj[:, :2]).min() - profile.footprint_radius))
        if gate is not None and gate.steps:
            interv.append(gate.interventions / gate.steps)
        if gate is not None and gate.model is not None:
            prims.append(len(gate.model.frames))
        if gate is not None:
            solves += getattr(gate, "corner_solves", 0)
            corner_s += getattr(gate, "corner_seconds", 0.0)
    return {"episodes": len(todo), "reached": c["reached"], "collision": c["collision"], "timeout": c["timeout"],
            "init_failed": c["init_failed"],
            "other": len(todo) - c["reached"] - c["collision"] - c["timeout"] - c["init_failed"],
            "min_clearance_m": round(clr, 4), "intervention_rate": round(float(np.mean(interv)) if interv else 0.0, 3),
            "median_time_to_goal_s": round(float(np.median(t_goal)) if t_goal else 0.0, 1),
            "primitives_last": round(float(np.mean(prims)) if prims else 0.0, 1),
            "ms_per_step": round(1000 * (time.perf_counter() - t0) / max(steps, 1), 1),
            "corner_solves": solves, "corner_ms_per_step": round(1000 * corner_s / max(steps, 1), 1)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--v0", nargs="+", default=None, help="V0 files (default: the profile's base_value)")
    ap.add_argument("--placements", nargs="+", default=["inward"], choices=list(PLACEMENTS))
    ap.add_argument("--worlds", nargs="+", default=["hj", "pillar", "occluded"], choices=["hj", "pillar", "occluded"])
    ap.add_argument("--unknowns", nargs="+", default=["occupied"], choices=["occupied", "free"])
    ap.add_argument("--nominals", nargs="+", default=["mppi"], choices=["mppi", "pursuit"])
    ap.add_argument("--filters", nargs="+", default=["backup"], choices=["backup", "hj", "hj_backup", "none"])
    ap.add_argument("--ranges", nargs="+", type=float, default=[None],
                    help="LiDAR ranges (m) instead of the profile's")
    ap.add_argument("--corners", action="store_true", help="correct the leaking corners (docs/13)")
    ap.add_argument("--corner-theta", type=int, default=30, help="headings of the corner windows (default 30)")
    ap.add_argument("--corner-frontier", action="store_true",
                    help="also correct the corners with a frontier edge (default: only between obstacle faces)")
    ap.add_argument("--corner-mode", default="sync", choices=["sync", "delayed", "thread"],
                    help="when the corrected corners can be used: at once, after their solve time (simulated), "
                         "or from a background thread")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parents[1] / "outputs" / "filter_batch.csv"))
    args = ap.parse_args()

    base = load_profile("burger")
    all_worlds = worlds(base)
    v0s = [load_v0(path) for path in args.v0 or [base.base_value]]
    # without the filter, the V0, placement and mode do not matter: one batch per world, nominal and range
    combos = [(w, r, n, "none", None, None, None) for w in args.worlds for r in args.ranges for n in args.nominals
              if "none" in args.filters]
    combos += [(w, r, n, f, v0, pl, u) for w in args.worlds for r in args.ranges for n in args.nominals
               for f in args.filters if f != "none" for v0 in v0s for pl in args.placements for u in args.unknowns]
    rows = []
    for wname, lidar, nominal, kind, v0, placement, unknown in combos:
        profile = base if lidar is None else dataclasses.replace(base, sensor=dataclasses.replace(base.sensor, range=lidar))
        row = {"world": wname, "range_m": profile.sensor.range, "nominal": nominal,
               "filter": "none" if v0 is None else f"{kind}, unknown {unknown}",
               "v0": "" if v0 is None else f"{v0.encoding}, {v0.meta.get('accuracy', '?')}, alpha {v0.alpha:g}",
               "placement": "" if v0 is None else placement,
               "corners": ((args.corner_mode + (", with frontier" if args.corner_frontier else ""))
                           if args.corners else "") if v0 is not None else "",
               **run(profile, all_worlds[wname], v0, PLACEMENTS.get(placement, {}), unknown or "occupied", nominal,
                     kind, corner_kw=dict(corners=True, corner_theta=args.corner_theta, corner_mode=args.corner_mode,
                                    corner_frontier=args.corner_frontier)
                     if args.corners else None)}
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
