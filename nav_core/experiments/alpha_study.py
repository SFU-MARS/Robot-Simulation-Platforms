"""
Choosing the primitive's alpha (constant-speed Dubins 3D, inward placement).

The primitive is a rectangle of length L and thickness L / sqrt(alpha).  With
the inward placement its free-facing side lies on the detected face, so the
thickness only matters through

  resolution -- the thin rectangle must be resolved by the grid of V_0: the
                transported value on the coarse online grid is compared with
                the same primitive solved on a finer grid, near the BRT boundary;
  geometry   -- behind a thin obstacle (depth d < L / sqrt(alpha)) the
                rectangle pokes out of the back;
  closedloop -- navigation with the backup filter for each alpha.

Legacy: this is how docs/10 (sections 10.4-10.5) was produced, with the code before the
backup filter port.  It needs Robot-Sim at daac2c4 (``FastBrtGatekeeper``) and
Hu-Hanyang/FaST-BRT on PYTHONPATH (its numpy solver for V_0), e.g. in a worktree:

    git worktree add /tmp/robot-sim-daac2c4 daac2c4 && cd /tmp/robot-sim-daac2c4/nav_core

The ported filter (``nav_core.safety.BackupFilter``) reads an exported V0 instead
(``python -m fast_brt export-v0 --alpha ...`` in fast-brt, one file per alpha).

Usage (from nav_core/ of that checkout):

    PYTHONPATH=.:<FaST-BRT> python experiments/alpha_study.py resolution
    PYTHONPATH=.:<FaST-BRT> python experiments/alpha_study.py geometry
    PYTHONPATH=.:<FaST-BRT> python experiments/alpha_study.py closedloop
    PYTHONPATH=.:<FaST-BRT> python experiments/alpha_study.py figures

Outputs: outputs/alpha/*.csv, outputs/alpha/*.png
"""

from __future__ import annotations

import argparse
import collections
import csv
import time
from pathlib import Path

import numpy as np

ALPHAS = (5.0, 10.0, 20.0, 40.0, 80.0)
OUT = Path(__file__).resolve().parents[1] / "outputs" / "alpha"
HORIZON = 2.0                      # model time units, as FastBrtGatekeeper
COARSE = (121, 60)                 # grid of the online V_0 (FastBrtGatekeeper): x/y points, theta points
FINE = (181, 60)                   # reference grid, 1.5x finer in x and y
EXTENT = 6.0                       # canonical grid [-EXTENT, EXTENT]^2 (model units)


def write_csv(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {path}")


def solve_v0(alpha: float, n_xy: int, n_theta: int):
    """V_0 at the full horizon on [-EXTENT, EXTENT]^2 (one slice kept)."""
    from fast_brt.hj import DubinsHJ, GridNP
    from fast_brt.primitive import BaseValue
    dyn = DubinsHJ(v_range=(1.0, 1.0), w_range=(-1.0, 1.0), role="avoid")
    base = BaseValue(GridNP.make((-EXTENT, EXTENT), (-EXTENT, EXTENT), n_xy, n_xy, n_theta), dyn, alpha,
                     HORIZON, HORIZON)
    return base.grid, base.slices[-1].astype(float)


def resolution(args):
    """Coarse (online) vs fine V_0 - L^2/4: sign agreement near the BRT boundary and BRT area."""
    lengths = (0.5, 1.0, 2.0, 4.0, 6.0)          # model units (x 0.1 m for the Burger)
    rows = []
    for alpha in ALPHAS:
        t0 = time.perf_counter()
        gc, vc = solve_v0(alpha, *COARSE)
        gf, vf = solve_v0(alpha, *FINE)
        secs = time.perf_counter() - t0
        X, Y, TH = gf.mesh()
        pts = np.stack([X.ravel(), Y.ravel(), TH.ravel()], 1)
        vc_at_fine = gc.interp(vc, pts).reshape(vf.shape)          # what the online filter reads
        cell = gf.dx * gf.dy
        for L in lengths:
            q = L * L / 4.0
            ref, est = vf - q, vc_at_fine - q
            brt_ref, brt_est = ref <= 0, est <= 0
            # boundary band: within two coarse cells of the reference BRT boundary (in the linear encoding)
            lin = np.sqrt(np.maximum(ref + q, 0.0)) - L / 2.0
            band = np.abs(lin) <= 2.0 * gc.dx
            rows.append({
                "alpha": alpha, "L_model": L, "L_burger_m": round(0.1 * L, 3),
                "thickness_model": round(L / np.sqrt(alpha), 4),
                "thickness_coarse_cells": round(L / np.sqrt(alpha) / gc.dx, 2),
                "brt_area_ref": round(float(brt_ref.mean(axis=2).sum() * cell), 4),
                "brt_area_rel_err": round(float((brt_est.mean(axis=2).sum() - brt_ref.mean(axis=2).sum())
                                                / max(brt_ref.mean(axis=2).sum(), 1e-9)), 4),
                "sign_mismatch_in_band": round(float((brt_ref != brt_est)[band].mean()), 4),
                "solve_s": round(secs, 1),
            })
            print(f"alpha {alpha:5.1f} L {L:3.1f}: thickness {L / np.sqrt(alpha):.3f} "
                  f"({L / np.sqrt(alpha) / gc.dx:.1f} coarse cells), BRT area err {rows[-1]['brt_area_rel_err']:+.3f}, "
                  f"sign mismatch near boundary {rows[-1]['sign_mismatch_in_band']:.3f}", flush=True)
    write_csv(OUT / "resolution.csv", rows)


def geometry(args):
    """Thickness L / sqrt(alpha) behind the face, and how far it pokes out of a thin obstacle."""
    lengths_m = (0.1, 0.2, 0.4, 0.9, 1.4)        # faces seen in the test worlds
    depth_m = 0.1                                # screens of occluded_4x4 are 0.1 m deep
    rows = []
    for alpha in ALPHAS:
        for L in lengths_m:
            t = L / np.sqrt(alpha)
            rows.append({"alpha": alpha, "L_m": L, "thickness_m": round(t, 3),
                         "pokes_out_of_0.1m_wall_m": round(max(0.0, t - depth_m), 3)})
    write_csv(OUT / "geometry.csv", rows)
    print(f"{'L (m)':>8s} " + " ".join(f"a={a:<5g}" for a in ALPHAS) + "   thickness (m)")
    for L in lengths_m:
        print(f"{L:8.2f} " + " ".join(f"{L / np.sqrt(a):7.3f}" for a in ALPHAS))


def _cases(world, profile, goals, n_starts, seed):
    rng = np.random.default_rng(seed)
    starts = []
    while len(starts) < n_starts:
        x = np.r_[rng.uniform(*world["xy"], 2), rng.uniform(-np.pi, np.pi)]
        if world["w"].distance(x[None, :2])[0] > profile.footprint_radius + 0.12:
            starts.append(x)
    return [(x0, g) for x0 in starts for g in goals if np.linalg.norm(x0[:2] - g) > 0.4]


def closedloop(args):
    """MPPI + backup filter (unknown occupied) for each alpha, inward placement, no cutting;
    baseline: the earlier centered placement with cutting at alpha 20."""
    from nav_core.mapping import OccupancyGrid
    from nav_core.nominal import make_nominal
    from nav_core.robots import ROBOTS_DIR, load_profile
    from nav_core.safety import FastBrtGatekeeper
    from nav_core.sim import Box, World, run_episode

    p = load_profile("burger")
    root = ROBOTS_DIR.parent
    hj = World.from_sdf(root / p.world, plane_z=p.sensor.height)
    occ = World.from_sdf(root / "turtlebot3/turtlebot3_controls/worlds/occluded_4x4.world", plane_z=p.sensor.height)
    walls = [b for b in hj.boxes if max(b.half) > 0.9]
    worlds = {
        "hj": {"w": hj, "xy": (-0.85, 0.85), "goals": [(0.7, 0.2), (0.65, 0.8), (-0.6, 0.6)], "n": 10, "grid": None},
        "pillar": {"w": World(walls + [Box((0.0, 0.0), (0.15, 0.15))]), "xy": (-0.85, 0.85),
                   "goals": [(0.6, 0.6), (-0.6, 0.6), (0.6, -0.6), (-0.6, -0.6)], "n": 8, "grid": None},
        "occluded": {"w": occ, "xy": (-1.8, 1.8), "goals": [(1.7, 0.6), (-1.6, -0.4), (0.9, -1.6), (-0.3, 1.7)],
                     "n": 6, "grid": occ.bounds()},
    }
    configs = [("centered+cut a=20", dict(alpha=20.0, placement="centered", cut=True))]
    configs += [(f"inward a={a:g}", dict(alpha=a, placement="inward", cut=False)) for a in ALPHAS]
    rows = []
    for wname, wd in worlds.items():
        cases = _cases(wd, p, [np.array(g) for g in wd["goals"]], wd["n"], seed=1)
        for cname, kw in configs:
            c, clr, interv, prims, t_goal, steps, t0 = collections.Counter(), np.inf, [], [], [], 0, time.perf_counter()
            for i, (x0, goal) in enumerate(cases):
                gate = FastBrtGatekeeper(p, **kw)
                grid = None if wd["grid"] is None else OccupancyGrid(*wd["grid"], p.map.resolution)
                ep = run_episode(p, wd["w"], x0, goal, make_nominal(p, "mppi", seed=i), safety=gate,
                                 t_max=100, seed=i, grid=grid)
                c[ep.status] += 1
                steps += len(ep.cmds)
                if ep.status == "reached":
                    t_goal.append(ep.t)
                if len(ep.traj) > 1:
                    clr = min(clr, float(wd["w"].distance(ep.traj[:, :2]).min() - p.footprint_radius))
                if gate.steps:
                    interv.append(gate.interventions / gate.steps)
                if gate.model is not None:
                    prims.append(len(gate.model.frames))
            row = {"world": wname, "config": cname, "episodes": len(cases),
                   "reached": c["reached"], "collision": c["collision"], "timeout": c["timeout"],
                   "init_failed": c["init_failed"], "other": sum(c.values()) - c["reached"] - c["collision"]
                   - c["timeout"] - c["init_failed"],
                   "min_clearance_m": round(clr, 4), "intervention_rate": round(float(np.mean(interv)) if interv else 0.0, 3),
                   "median_time_to_goal_s": round(float(np.median(t_goal)) if t_goal else 0.0, 1),
                   "primitives_last": round(float(np.mean(prims)) if prims else 0.0, 1),
                   "ms_per_step": round(1000 * (time.perf_counter() - t0) / max(steps, 1), 1)}
            rows.append(row)
            print(" | ".join(f"{k} {v}" for k, v in row.items()), flush=True)
    write_csv(OUT / "closedloop.csv", rows)


def figures(args):
    """V_0 BRTs for each alpha (inward placement), and the primitives on a map, centered+cut vs inward."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    OUT.mkdir(parents=True, exist_ok=True)
    # (1) BRT of one face of length L = 4 (0.4 m) for every alpha in one plot; face on y = 0, free above,
    # inward placement (rectangle below the face). Exact: heading at the face, the BRT reaches one
    # turning radius (y = 1) in front of the face, except within one radius of the ends.
    L = 4.0
    colours = dict(zip(ALPHAS, ["tab:purple", "tab:blue", "tab:green", "tab:orange", "tab:red"]))
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.4), squeeze=False)
    for ax, (frac, title) in zip(axes[0], ((0.75, "heading -y (at the face)"), (0.0, "heading +x (along the face)"))):
        for alpha in ALPHAS:
            g, v = solve_v0(alpha, *COARSE)
            hw = L / (2 * np.sqrt(alpha))
            k = int(round(frac * len(g.thetas))) % len(g.thetas)
            ax.contour(g.xs, g.ys - hw, (v[:, :, k] - L * L / 4.0).T, levels=[0.0], colors=colours[alpha],
                       linewidths=1.3)
            ax.add_patch(Polygon(np.array([[-L / 2, 0], [L / 2, 0], [L / 2, -2 * hw], [-L / 2, -2 * hw]]),
                                 fill=False, ec=colours[alpha], lw=0.6, ls="--"))
            ax.plot([], [], color=colours[alpha], label=f"alpha {alpha:g} (thickness {2 * hw:.2f})")
        if frac == 0.75:                                   # exact front: h*(d) = sqrt(1 - (1 - d)^2), d = distance to the nearer end
            x = np.linspace(-L / 2, L / 2, 401)
            d = L / 2 - np.abs(x)
            ax.plot(x, np.where(d >= 1, 1.0, np.sqrt(np.clip(1 - (1 - d) ** 2, 0, None))), "k:", lw=1.5,
                    label="exact BRT front")
        ax.plot([-L / 2, L / 2], [0, 0], "k", lw=2.5)
        ax.set_title(title, fontsize=10)
        ax.set_xlim(-3.5, 3.5); ax.set_ylim(-2.2, 1.6); ax.set_aspect("equal"); ax.grid(alpha=0.3)
    axes[0][0].legend(loc="lower left", fontsize=7)
    fig.suptitle(f"BRT (zero level) of one face, L = {L:g} model units = {0.1 * L:g} m, inward placement "
                 "(black: detected face, dashed: primitive rectangle)", fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / "brt_vs_alpha.png", dpi=130)
    print(f"wrote {OUT / 'brt_vs_alpha.png'}")

    # (2) primitives on the occluded world after part of a run: centered+cut vs inward (alpha 20)
    from nav_core.mapping import FREE, OCCUPIED, OccupancyGrid
    from nav_core.nominal import make_nominal
    from nav_core.robots import ROBOTS_DIR, load_profile
    from nav_core.safety import FastBrtGatekeeper
    from nav_core.sim import World, run_episode

    p = load_profile("burger")
    world = World.from_sdf(ROBOTS_DIR.parent / "turtlebot3/turtlebot3_controls/worlds/occluded_4x4.world",
                           plane_z=p.sensor.height)
    o, s = world.bounds()
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.4), squeeze=False)
    for ax, (name, kw) in zip(axes[0], (("centered + cut, alpha 20", dict(alpha=20.0, placement="centered", cut=True)),
                                        ("inward, no cut, alpha 20", dict(alpha=20.0, placement="inward", cut=False)))):
        gate = FastBrtGatekeeper(p, **kw)
        ep = run_episode(p, world, (-0.9, -1.4, 0.0), (0.9, -0.3), make_nominal(p, "mppi"), safety=gate,
                         t_max=14.0, grid=OccupancyGrid(o, s, p.map.resolution))
        g = ep.grid
        st = g.state()
        img = np.full(st.shape, 0.75)
        img[st == FREE], img[st == OCCUPIED] = 1.0, 0.0
        ext = [g.origin[0], g.origin[0] + g.nx * g.res, g.origin[1], g.origin[1] + g.ny * g.res]
        ax.imshow(img, origin="lower", extent=ext, cmap="gray", vmin=0, vmax=1, interpolation="nearest")
        for b in world.boxes:
            c, sn = np.cos(b.yaw), np.sin(b.yaw)
            loc = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * b.half
            ax.add_patch(Polygon(loc @ np.array([[c, sn], [-sn, c]]) + b.center, fill=False, ec="tab:red", lw=0.6, ls="--"))
        for center, hl, hw, beta in gate.model.frames:
            c, sn = np.cos(beta), np.sin(beta)
            loc = np.array([[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]])
            ax.add_patch(Polygon((loc @ np.array([[c, sn], [-sn, c]]) + center) * p.units.length,
                                 fc="tab:blue", alpha=0.35, ec="tab:blue", lw=0.6))
        for ring in gate.model.rings:
            ax.add_patch(Polygon(ring * p.units.length, fill=False, ec="tab:purple", lw=0.8))
        ax.plot(ep.traj[:, 0], ep.traj[:, 1], "k", lw=1.0)
        ax.set_title(f"{name}: {len(gate.model.frames)} primitives (blue), region (purple)", fontsize=9)
        ax.set_xlim(-1.6, 1.0); ax.set_ylim(-2.1, 0.5); ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(OUT / "primitives_on_map.png", dpi=130)
    print(f"wrote {OUT / 'primitives_on_map.png'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("part", choices=["resolution", "geometry", "closedloop", "figures"])
    args = ap.parse_args()
    {"resolution": resolution, "geometry": geometry, "closedloop": closedloop, "figures": figures}[args.part](args)


if __name__ == "__main__":
    main()
