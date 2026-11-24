# CLAUDE.md

Shared conventions for working in this repository with Claude Code. The detailed record is in [`docs/`](docs/), written in Chinese. Read [`docs/11_roadmap.md`](docs/11_roadmap.md) first: it has the decisions made so far, what is done, and the next steps.

## What this repository is

The robot navigates an **unknown** map with a nominal controller (pursuit or MPPI) and builds an occupancy map from LiDAR while driving. Safety comes from HJ reachability on **2D line primitives**: one value function per primitive is solved offline, then transported to each detected obstacle face online. An online **local update** fixes the corners where the union of primitives leaks.

Robots:
- TurtleBot3 Burger: Dubins3D, constant speed; this one is ready.
- QBot (Quanser QBot Platform): Dubins4D (x, y, θ, v), control (a, w); offline V0 and the ROS-free closed loop are done (docs/15), corner correction and ROS not yet.
- Crazyflie: 6D; placeholder.

## Where code goes

| Code | Repository |
| --- | --- |
| Primitives, value functions, offline solves, local update | `fast-brt/` (submodule, SFU-MARS/fast-brt, branch `robot-sim`) |
| Leaking-corner local update kernels | `fast-brt/HJ_local_update/` (submodule of fast-brt, SFU-MARS/HJ_local_update, branch `robot-sim`) |
| Online leaking-corner correction (stationary local update, numpy) | `fast-brt/fast_brt/corner.py`; used by `nav_core/safety/scene.py` (`corners=True`), see `docs/13_online_local_update.md` |
| Robot profiles, mapping, obstacle detection, nominal controllers, safety filters, simulation | `nav_core/` (this repository) |
| ROS 2 integration | `ros2/fastbrt_nav/` (this repository) |

- **Dependency direction.** Dependencies point one way: this repository → fast-brt → HJ_local_update. Don't copy HJ code into this repository.
- **Reuse first.** Reuse the existing code in fast-brt and HJ_local_update where possible. Add code there only when it is missing, and keep it in the same structure.
- **Ownership.**
  - Dynamics (4D, 6D) are owned by a labmate.
  - In `HJ_local_update` the new behaviour must stay opt-in (for example `brt=False` by default). Check every change against a direct solve.
- **Offline V0.** The backup filter (`nav_core/safety/backup_filter.py`) reads the robot's offline V₀ (`base_value` in `robots/<name>.yaml`, exported with `python -m fast_brt export-v0` on mars-14; not in git: assets of a GitHub release, listed with sha256 in `robots/v0_release.yaml`, fetched into `fast-brt/results/v0/` by `nav_core/scripts/fetch_v0.py`; a new V₀ gets a new release tag) through fast-brt's numpy-only `fast_brt.query`, imported from `PYTHONPATH` or the `fast-brt/` submodule. The old `Hu-Hanyang/FaST-BRT` is retired; never put it on `PYTHONPATH` (same package name).

## Git rules

- **Never push to `main` of SFU-MARS/fast-brt or SFU-MARS/HJ_local_update.** Those are the labmates' repositories. Push only our branch `robot-sim`:
  - name the refspec explicitly (`git push origin robot-sim:robot-sim`);
  - run `--dry-run` first.
- This repository works on branch `add_lidar`.
- **Order after a submodule change:**
  1. commit and push HJ_local_update;
  2. commit and push fast-brt, then bump its pointer to HJ_local_update;
  3. bump the `fast-brt` pointer here.
  
  Check with a fresh `git clone --recurse-submodules`.
- **Committing.** Don't commit or push unless asked. Commit messages are in English.

## Environment and commands

- **`nav_core`:**
  - ROS-free; never import `rclpy` in it;
  - keep it compatible with Python 3.10 and numpy 1.21 (ROS 2 Humble);
  - numpy, scipy, contourpy and pyyaml only.
- **odp:**
  - `optimized_dp` (odp) solves need HeteroCL and run offline on the desktop (mars-14, Python 3.8). Scripts meant for it must stay Python 3.8 compatible.
  - Online code may import odp's `Grid` / index utilities, which don't need HeteroCL.
  - fast-brt needs the old `Grid(min, max, dims, pts, pDim)` signature, i.e. an optimized_dp version from before commit 689548e.

```bash
cd nav_core
python scripts/fetch_v0.py                                             # download the V0 files (GitHub release)
PYTHONPATH=. python -m pytest -q tests                                  # nav_core tests (closed-loop filter tests skip without the V0 file; NAV_CORE_REQUIRE_V0=1 makes them fail)
PYTHONPATH=. python scripts/run_sim.py --nominal mppi                   # ROS-free closed loop
PYTHONPATH=. python scripts/run_sim.py --nominal mppi --filter backup

cd fast-brt
python -m pytest -q tests                                               # quick tests; -m slow needs HeteroCL
```

## Conventions

- **Units.** HJ code works in nondimensional units: v = 1, ω_max = 1, turning radius 1. `nav_core.robots` converts physical ↔ model units (Burger: 1 unit = 0.1 m, 1 s).
- **Primitives:**
  - **inward placement**: the rectangle's free-facing side lies on the detected face;
  - linear encoding `max(|x|, √α|y|) − L/2`, α = 20;
  - **BRT** (`minVWithV0`), not BRS;
  - faces shorter than `L_min ≈ 2·dx·√α` are extended;
  - the offline V₀ uses odp `accuracy="low"` on a 241 grid (`robots/burger.yaml` → `v0_dubins_3d_linear_alpha20_low.npz`): the local update recomputes first-order steps, so everything that feeds it must be `low`;
  - `medium` V₀ and the quadratic encoding are kept as options for comparisons (point query only, not the local update).
- **Obstacles.** Touching obstacles count as one obstacle.
- **Filter modes.** Three are kept as options: `hj`, `backup` and `hj_backup`. The "witness gatekeeper" is called the **backup filter** in new code and docs.
- **Docs:**
  - `docs/` is Chinese prose with English technical terms and variable names;
  - keep the module docs (01–11) up to date when behaviour changes;
  - don't reintroduce the old sandwich certificate / B&B material.
