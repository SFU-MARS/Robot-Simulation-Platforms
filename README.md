# Robot Simulation Platforms

This repository follows the [Fast-BRT Experiment Document](https://docs.google.com/document/d/1S5lzXlQSW35xeCZrHe30NN7NRfxiYE0QOhNOWc51FYE/edit?tab=t.0). Most folders are git submodules. Clone them explicitly by adding `--recurse-submodules` to `git clone`, or run `git submodule update --init --recursive` after a checkout.

**[`docs/`](docs/) walks through the code module by module (in Chinese), ending with one full MPPI control period from loading the map to the 2D primitives.** Progress, decisions and next steps are in [`docs/11_roadmap.md`](docs/11_roadmap.md). Conventions for working with Claude Code are in [`CLAUDE.md`](CLAUDE.md). This README stays the quick reference: setup, usage, results and TODO.

## Goal

The robot navigates an **unknown** map: it does not know the obstacles in advance. It senses them with a LiDAR, builds its own map while driving, and reaches a goal. An HJ reachability safety filter keeps it safe on the way.

Each control period:

```
/scan + /odom ──► mapping: accumulated occupancy grid ──► boundary edges (obstacle | frontier)
                                                                │  edge length = obstacle side estimate
/odom ──► nominal controller (obstacle-unaware) ──► u_nom       ▼
                                                  safety filter (2D primitives, BRT) ──► /cmd_vel
```

- **Nominal controller.** It drives towards the goal and carries no safety guarantee. Two are available: pure pursuit, which ignores the map, and MPPI on the accumulated map, which is optimistic about unknown space.
- **Safety filter.** There is one 2D line primitive per obstacle face, transported from a single precomputed BRT value function. The filter overrides the nominal control only when needed. By default unknown space is treated as occupied.
  - Where two primitives meet at a corner, their union can leak. An online local update ([HJ_local_update](https://github.com/SFU-MARS/HJ_local_update)) is planned to fix this.
  - Three filter modes: `hj` (classic HJ filter, not safe at leaking corners), `backup` (backup filter) and `hj_backup` (HJ first, backup as the safety net); see [docs/05](docs/05_safety_filter.md) §5.8.
- **Map.** It accumulates over the whole run. The environment is static, so the known free space only grows. In simulation the pose is known (odometry / ground truth), so no SLAM is needed.

### Division of work

| | [fast-brt](https://github.com/SFU-MARS/fast-brt) (submodule `fast-brt/`, branch `robot-sim`) | this repository |
| --- | --- | --- |
| Scope | dynamics, primitives, HJ value functions (offline solves), leaking-corner local update ([HJ_local_update](https://github.com/SFU-MARS/HJ_local_update), submodule `fast-brt/HJ_local_update/`, branch `robot-sim`) | navigation: robot profiles, mapping and obstacle detection, nominal controllers, safety filters, ROS 2 / Gazebo integration |
| Depends on | numpy / scipy; `optimized_dp` + HeteroCL for solves | fast-brt, ROS 2 |

- **Dependency direction.** Dependencies point one way: this repository → fast-brt → HJ_local_update. No HJ solver code lives here.
- **Branches.** Both submodules track our own branch `robot-sim`. Never push to their `main`.
- **Offline V0, online point query.** The backup filter reads one offline value function V₀ (`python -m fast_brt export-v0` in fast-brt, on the odp machine) with fast-brt's `fast_brt.query`, which is numpy only. The V₀ file is set per robot (`base_value` in `robots/<name>.yaml`, a path from the repository root). It is not in git: the V₀ files are assets of a GitHub release of this repository, listed with their sha256 in `robots/v0_release.yaml`; `cd nav_core && python scripts/fetch_v0.py` downloads them to `fast-brt/results/v0/`, and loading a listed file checks its checksum. `ROBOT_SIM_V0=<file>` overrides the profile's V₀. `nav_core.safety` imports `fast_brt.query` from `PYTHONPATH`, or else from the `fast-brt/` submodule. The old [Hu-Hanyang/FaST-BRT](https://github.com/Hu-Hanyang/FaST-BRT) is no longer used; don't put it on `PYTHONPATH` (its package has the same name).
- **No online solves.** Primitive values are solved offline with `optimized_dp` and loaded at start-up. The local update recomputes only a small region online: HeteroCL in the `odp` environment, or the numpy implementation.
- **odp utilities online.** Online code may import `odp`'s grid / index utilities, which do not need HeteroCL. It reaches them through `PYTHONPATH`.

### Robots

| Robot | Dynamics | Simulator | Status |
| --- | --- | --- | --- |
| TurtleBot3 Burger | Dubins 3D (x, y, θ), constant speed | Gazebo | profile ready |
| Quanser QBot | Dubins 4D (x, y, θ, v) | Gazebo | placeholder |
| Crazyflie | 6D | TBD | placeholder |

Each robot has one profile, `robots/<name>.yaml`. It is the single source of the robot's parameters:
- dynamics name and physical limits;
- footprint;
- sensor: type, range, rate;
- map extent and resolution;
- world file and precomputed value file.

The map size and the LiDAR range are therefore chosen per robot.

## Repository layout

```
Robot-Simulation-Platforms/
  CLAUDE.md                     # conventions for Claude Code (repo split, git rules, environment)
  docs/                         # module-by-module docs (Chinese), roadmap in 11_roadmap.md
  fast-brt/                     # submodule SFU-MARS/fast-brt (branch robot-sim): primitives, offline values, local update
    HJ_local_update/            #   its submodule SFU-MARS/HJ_local_update (branch robot-sim)
  robots/                       # per-robot profiles: burger.yaml, qbot.yaml, crazyflie.yaml
  nav_core/                     # ROS-free navigation core (pip-installable project)
    pyproject.toml
    nav_core/                   # the importable package
      robots/                   #   profile loading, physical <-> model units
      mapping/                  #   occupancy grid, region, boundary edges, obstacle layer (faces, corners)
      nominal/                  #   nominal controllers: pursuit, map-aware MPPI
      sim/                      #   ROS-free closed loop: SDF world, synthetic LiDAR, unicycle plant
      safety/                   #   safety filters as hooks (unit conversion, map -> scene, primitive placement)
    scripts/run_sim.py
    experiments/                #   e.g. alpha_study.py (primitive placement and alpha, docs/10)
    tests/
  turtlebot3/
    turtlebot3_controls/        # ROS 2 package: worlds (hj_reach_target, worlds/occluded_4x4), launch, bridge params, legacy scripts
    turtlebot3_simulations/     # submodule (ROBOTIS), not modified
  ros2/fastbrt_nav/             # ROS 2 node: /scan + /odom -> nav_core + safety filter -> /cmd_vel, RViz map and markers
  mujoco/unitree_mujoco/        # submodule (Unitree, MuJoCo)
  unitree_mujoco/               # submodule (same commit as above)
```

Planned addition: our own copy of the robot SDF models, so that the LiDAR range and rate can be set per profile without modifying `turtlebot3_simulations`.

`nav_core` deliberately does not import `rclpy`. All robots and simulators share it, and it is tested without ROS.

## nav_core

Requirements: Python ≥ 3.10, `numpy`, `scipy`, `contourpy`, `pyyaml`. It is tested with ROS 2 Humble's versions (numpy 1.21.5, scipy 1.8.0) and with current versions. Ubuntu 22.04 has no apt package for `contourpy`. Install it with `pip install "contourpy<1.2"`, which leaves the system numpy alone.

```
cd nav_core
PYTHONPATH=. python -m pytest -q tests
```

Minimal use:

```python
from nav_core.robots import load_profile
from nav_core.mapping import OccupancyGrid, boundary_edges

profile = load_profile("burger").require_ready()
grid = OccupancyGrid.from_profile(profile)
grid.integrate_scan(pose, angles, ranges, profile.sensor.range)   # every scan, world-frame pose (x, y, theta)
edges = boundary_edges(grid)                                      # side-length estimates (point robot)
safe_edges = boundary_edges(grid, footprint_radius=profile.footprint_radius)   # for the safety filter
```

- **Edge labels.** An `obstacle` edge has an observed occupied cell behind it. A `frontier` edge has unknown space behind it.
- **Accuracy.** In the test world, obstacle side lengths are recovered to within about one cell (2 cm). Marching squares chamfers convex corners, and these corners are snapped back by intersecting the adjacent sides.
- **Corner clearance.** At convex corners the free region can exceed the true free space by less than one cell. The safety clearance should include at least one cell.

**ROS-free closed loop.** `nav_core.sim` stands in for one Gazebo run. It reads the collision boxes of the same SDF world, keeping those that span the LiDAR plane. It ray-casts a synthetic scan that follows the `LaserScan` conventions and integrates a unicycle exactly. Every control period it runs scan → map → nominal → optional safety hook `safety(x, w_nom, grid) -> w`. The safety filter plugs into this hook.

```
cd nav_core
PYTHONPATH=. python scripts/run_sim.py                   # burger, hj_reach_target.world, (-0.8, -0.8, 0.2) -> (0.7, 0.2)
PYTHONPATH=. python scripts/run_sim.py --noise 0.01      # 1 cm range noise
```

With the nominal controller only, the Burger grazes the corner of obstacle 2 and collides at t ≈ 7.3 s. This is the case the safety filter has to handle. Figures are written to `nav_core/outputs/`.

**Backup filter (formerly "witness gatekeeper").** `nav_core.safety.BackupFilter` is the safety hook (`--filter backup`); the witness search is in `nav_core.safety.witness`. Each period it does three things:
1. It takes the region the robot may use from the accumulated map, shrunk by the footprint (`free_region`). The region is an outer boundary plus one closed ring around every obstacle inside it.
2. It puts one primitive on every obstacle face (plus the frontier edges with `unknown=occupied`), placed inward, with its value from V₀ (`fast_brt.query`).
3. It accepts the nominal turn rate only if a witness exists from the next state: a rollout that keeps the clearance and ends on a free loiter circle. The first candidate of the search is the HJ control of the nearest primitive.

Two modes (`unknown=`), to be compared:
- `occupied` (default): unknown space is an obstacle. The region is the known free space, so frontier edges are obstacles too. This mode is safe even against obstacles hidden behind others.
- `free`: only detected obstacles count. The region is everything but the detected obstacles. It is less conservative, but it is safe only if every obstacle is seen before the robot enters its BRT.

In both modes the modelled obstacle is the complement of the region. Every detected obstacle cell, grown by the footprint, is therefore covered completely, not just outlined by thin primitives. A free-standing obstacle (a pillar, a rock) is enclosed by its own ring. This holds even when only one face has been seen (`nav_core.safety.Scene`).

```
PYTHONPATH=. python scripts/run_sim.py --filter backup [--unknown free]
```

Burger, default start and goal (pursuit): the run reaches the goal at t ≈ 17.9 s, with 12 interventions in 90 periods and a 4.3 cm minimum clearance. Batches of random starts × goals (`experiments/filter_batch.py`, pursuit, the default V₀ and inward placement, robot radius 0.124 m, 2026-11-23) gave these results (no collision with either filter mode):

| world | | reached | collision | timeout |
| --- | --- | --- | --- | --- |
| `hj_reach_target` (26 runs) | nominal only | 8 | 18 | 0 |
| | unknown occupied | 15 | 0 | 11 |
| | unknown free | 15 | 0 | 11 |
| walls + free-standing pillar (28 runs) | nominal only | 17 | 11 | 0 |
| | unknown occupied | 28 | 0 | 0 |
| | unknown free | 28 | 0 | 0 |

- **Modes look alike here.** The two modes barely differ because the 2 m arena with a 0.8 m LiDAR range has little occlusion. A larger, more occluded world is needed to compare them.
- **Timeouts.** Pure pursuit ignores obstacles, so the backup filter keeps rejecting it and the robot loiters on its certified circle. This is a liveness problem of the nominal controller, not of the filter.
- **Geometry.** The 2 m arena is tight for a certified loiter circle. What matters is the turning radius v / ω_max. At 0.2 m there is no witness at the start, so the Burger profile uses 0.1 m (v = 0.1 m/s, ω_max = 1 rad/s). This is also within the differential-drive limit v + ω_max · track / 2 ≤ wheel speed, which `load_profile` checks. The Burger's 0.22 m/s and 2.84 rad/s maxima cannot be reached at the same time.
- **Primitive placement.** The results above use the default `placement="inward", cut=False`: the rectangle's free-facing side lies on the face, so the BRT depends only on the face length L and no cutting is needed (Design decision 3). The earlier `placement="centered", cut=True` (the face as the rectangle's midline, long faces cut so that no primitive reaches more than one cell off its face) is kept as an option; it also has no collisions, with more interventions and 3–4 times more primitives ([docs/10](docs/10_primitive_alpha_accuracy.md), [roadmap](docs/11_roadmap.md) §11.3).

**Nominal controllers.**
- `pursuit` always turns the robot's heading towards the goal, at a turn rate proportional to the heading error (capped at ω_max). It ignores the map, so it drives straight into obstacles.
- `mppi` samples many short turn-rate sequences, simulates each one, and blends them, favouring low cost. The cost has two parts:
  - the shortest-path distance to the goal through the known map, computed around obstacles;
  - a penalty for getting close to occupied cells.

  It is optimistic: unknown space counts as drivable.

Batches of random starts × goals, with no filter and with the backup filter:

| world | nominal | nominal only | + backup filter (unknown occupied) |
| --- | --- | --- | --- |
| `hj_reach_target` (26 runs) | pursuit | 8 reached, 18 collisions | 15 reached, 0 collisions, 11 timeouts (intervention rate 0.26) |
| | mppi | 19 reached, 7 collisions | 15 reached, 0 collisions, 11 timeouts (0.080) |
| walls + pillar (28 runs) | pursuit | 17 reached, 11 collisions | 28 reached (0.072) |
| | mppi | 28 reached | 28 reached (0.015) |

The runs are those of [docs/10](docs/10_primitive_alpha_accuracy.md) whose start is at least the robot radius + 0.12 m from the obstacles: with the radius 0.124 m (since 2026-11-23, before 0.105 m) one start of `hj_reach_target` drops out.

**Why the 11 timeouts in `hj_reach_target` do not go away with MPPI.** The robot model cannot stop: it always drives at constant speed and can only steer. The only way to "wait safely" is to drive in a tight circle (radius 0.1 m). The backup filter therefore accepts a command only if, afterwards, the robot still has a way to reach known free space wide enough to circle in. That takes about 0.49 m: the circle's 0.2 m diameter, plus the robot's width (2 × 0.124 m), plus a margin. The upper-left area of this world is reachable only through a 0.4 m gap between obstacle 1 and the north wall. The robot itself would fit, but it could not circle inside the gap, so the backup filter never lets it in. Exactly 11 of the 26 runs need that gap. This is a limit of the constant-speed model, not of the nominal controller. A robot that can brake (Dubins 4D, QBot) can "wait safely" by stopping and does not have it.

**Occluded world and the two filter modes.** `worlds/occluded_4x4.world` is a 4 × 4 m arena with:
- three screens, each with a box hidden right behind one of its ends;
- three free-standing pillars.

The sweep uses MPPI as the nominal controller and varies the LiDAR range. There are 23 runs per row: random starts × goals (`experiments/filter_batch.py --worlds occluded --ranges 0.3 0.5 0.8`, robot radius 0.124 m, 2026-11-23).

| LiDAR range | no filter | unknown = occupied | unknown = free |
| --- | --- | --- | --- |
| 0.3 m (3 × turning radius) | 20 reached, 3 collisions | cannot start (23 runs) | 15 reached, **8 collisions** |
| 0.5 m (5 ×) | 20 reached, 3 collisions | 16 reached, 0 collisions, 7 timeouts | 17 reached, 0 collisions, 6 timeouts |
| 0.8 m (8 ×) | 22 reached, 1 collision | 17 reached, 0 collisions, 6 timeouts | 17 reached, 0 collisions, 6 timeouts |

How to read the table:
- *`unknown = free` is not safe when the LiDAR range is short.* This mode treats unseen space as drivable, so the filter can approve a path into space that hides an obstacle. With a 0.3 m range, the obstacle is seen too late to turn away.
- *`unknown = occupied` never collides, but needs a minimum range.* After the first scan, the known free space must already hold the circle the robot waits on: roughly LiDAR range ≥ robot radius + 2 × turning radius + margin ≈ 0.36 m. At 0.3 m it cannot start.
- *At 5× and 8× the turning radius both modes are safe.* `free` was not faster (one run more at 0.5 m).

**Obstacle layer.** `extract_obstacles(grid, offset=0)` returns the obstacles of the accumulated map (README, Design decisions 1–4):
- the obstacles themselves: connected components of occupied cells, with touching objects merged into one;
- their faces: enclosing lines with a free-side normal;
- their corners: where two faces meet, flagged convex or concave with respect to the obstacle.

Results:
- *Test world, noise-free.* `hj_reach_target` gives one obstacle with 12 faces. Its 4 convex corners are exactly the outer corners of obstacles 1 and 2. Obstacle 1's sides come out as 0.43 / 0.50 / 0.53 / 0.39 m (true 0.4 × 0.5 m).
- *Enclosure.* Every face lies at least 0.95 cm outside the true obstacles. With `offset` = the footprint radius, the faces describe the configuration-space obstacle.
- *Noise.* With 1 cm range noise and 10 scans per place, there are 13 faces and 5 convex corners. Here a face can cut up to 0.6 cm into the true obstacle, because the map itself is noisy; the safety clearance (1.5 cells) covers this.

Backup filter comparison, primitives from faces vs from the region's rings: there are no collisions with either. Reached and timeout counts are identical. The minimum clearance grows from 3.4 to 5.5 cm (`hj_reach_target`) and from 2.5 to 4.8 cm (pillar world). This is because the faces enclose the obstacles with square corners. The cost stays at about 15–30 ms per step.

**Obstacle tracking and detection events.** `extract_obstacles` relabels the map from scratch every period, so its obstacle ids are not stable. `ObstacleTracker.update(obstacle_map, t)` fixes that:
- *Matching.* It matches each obstacle to the previous period's by shared occupied cells. Occupied cells persist: an obstacle grows as more of it is seen.
- *Relabelling.* It relabels the obstacle map with stable ids.
- *Events.* It reports what changed:
  - `new` — a first detection;
  - `updated` — the number of faces or corners changed, or a face endpoint moved by more than 2 cells; e.g. more of the obstacle was seen;
  - `merged` — two tracked obstacles touched; the merged obstacle keeps the older id;
  - `split` and `removed` — caused by noise.

  Each event carries the obstacle's faces (with lengths) and corners. Obstacles of fewer than 4 cells are tracked but not reported.
- *Purpose.* Downstream work (the leaking-corner update, logs) can then act only on the obstacles that changed.

Where it is used:
- `run_episode(..., tracker=ObstacleTracker())` collects the events, and `run_sim.py --events` prints them. In the occluded world, for example, screen_1 grows from 0.91 m to its true 1.40 m as the robot drives along it. The hidden box is detected at t ≈ 8.6 s and ends at faces of about 0.22 × 0.20 m (true 0.2 × 0.2 m).
- `BackupFilter` gives its faces stable ids and keeps the period's events in `gate.events`.
- The ROS 2 node publishes every event as JSON on `obstacle_events`.

**Map update model.** Each cell counts how often a ray *ended* in it (hit) and how often a ray *passed through* it (pass):
- *occupied* if hits ≥ 20 % of all its observations (τ = 0.2);
- *free* if it was observed with fewer hits than that;
- *unknown* if it was never observed.

Why counting rather than the classic additive log-odds score:
- *Surface cells are partly free.* A 2 cm cell on an obstacle surface is partly obstacle and partly free. Rays that graze its free part pass through it far more often than rays end in it. With log-odds (hit +0.85, pass −0.7, clipped at ±4) those passes wipe the cell back to unknown, and faces and corners flicker.
- *Low hit ratios still count.* A hit ratio of 20 % keeps such a cell occupied. A stray noisy hit in open space (1 hit among many passes) stays free.
- *No clipping.* Nothing is clipped, so evidence is never forgotten.

In addition, a cell that has been hit is counted as passed only by rays that go near its centre (within 0.3 cells), not by rays clipping its corner (`graze`).

Comparison on the occluded world (three routes, MPPI, no filter):

| map model | flicker events (0 / 1 / 2 cm noise) | false obstacle cells in open space (2 cm) | deepest face cut into a true obstacle (1 / 2 cm) |
| --- | --- | --- | --- |
| log-odds (previous default) | 38 / 80 / 112 | 0 | 3.0 / 2.0 cm |
| log-odds + graze | 25 / 61 / 76 | 0 | 3.0 / 1.5 cm |
| counts, τ = 0.1 | 0 / 6 / 23 | 31 | 0 / 4.5 cm |
| **counts, τ = 0.2 + graze (default)** | **2 / 15 / 27** | 5 | 0.3 / 0.6 cm |
| counts, τ = 0.3 | 6 / 19 / 35 | 0 | 3.0 / 1.0 cm |

A flicker event is an `updated` event in which an obstacle lost cells; in a static world an obstacle should only grow. The previous model is still available as `OccupancyGrid(..., model="logodds")`.

Switching the default did not change the batch results of that time except the occluded sweep, which moved by one or two runs. There were no collisions with either filter mode. (The batches above were rerun on 2026-11-22 with the ported filter and the new V₀.)

**Scan conventions** (REP 117):
- `+inf`, or a reading ≥ range max, means no return: the ray is free up to the range.
- `nan`, `-inf` and readings below `range_min` are invalid and are skipped. They never clear cells.

**Units.** The HJ Dubins 3D model is nondimensional (v = 1, |ω| ≤ 1, turning radius 1). For the Burger profile (v = 0.1 m/s, ω_max = 1 rad/s), one model length unit is 0.1 m and one model time unit is 1 s (`profile.units`).

## ROS 2 node: fastbrt_nav

`ros2/fastbrt_nav` runs the same loop as `nav_core.sim` against Gazebo (or a real robot):

```
/scan, /odom → accumulated map → nominal (pursuit | mppi) → safety filter (backup | hj | hj_backup) → /cmd_vel
```

Behaviour:
- It integrates the newest scan at the newest pose, once per control period (the profile's `ctrl_dt`).
- It publishes the backup filter's zero-order-hold turn rates on a sub-period timer.
- It drives at the profile's constant speed. It stops (zero `cmd_vel`) before the first scan, at the goal, and if the filter cannot certify the start or fails.
- It uses only `LaserScan`, `Odometry` and `Twist`, so the same code runs on Humble and Jazzy.

On ROS 2 Humble (Ubuntu 22.04) it uses Gazebo Classic (`turtlebot3_controls/launch/hj_launch_world_classic.launch.py`, launch argument `gazebo:=classic`, the default on Humble); on Jazzy the new Gazebo (`gazebo:=gz`). It has not run in Gazebo yet. Without ROS, the message conversions are unit-tested and `test/test_node_without_ros.py` runs the whole node (rclpy and messages stubbed) in the simulated world with every filter. The step-by-step desktop test is [docs/14](docs/14_ros2_desktop_test.md).

Build and run (from `turtlebot3/`, next to `turtlebot3_controls`):

```
export PYTHONPATH=<repo>/nav_core:<repo>/fast-brt:$PYTHONPATH     # nav_core and fast_brt are not ROS packages
colcon build --base-paths . ../ros2 --packages-select turtlebot3_controls fastbrt_nav
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch fastbrt_nav fastbrt_nav.launch.py                       # hj_reach_target, (-0.8, -0.8, 0.2) -> (0.7, 0.2)
ros2 launch fastbrt_nav fastbrt_nav.launch.py \
    world:=$(ros2 pkg prefix turtlebot3_controls)/share/turtlebot3_controls/worlds/occluded_4x4.world \
    x_pose:=-0.9 y_pose:=-1.4 yaw_pose:=0.0 goal:="[0.9, -0.3]" map_origin:="[-2.1, -2.1]" map_size:="[4.2, 4.2]"
```

Launch arguments:
- `robot` (profile, default `burger`), `goal`;
- `nominal` (`mppi` | `pursuit`), `filter` (`backup` | `hj` | `hj_backup` | `none`), `unknown` (`occupied` | `free`), `primitives` (`faces` | `rings`), `v0` (V₀ file; default the profile's `base_value`);
- `corners` (`auto`: on for `hj` | `true` | `false`), `corner_mode` (`thread`, the corner windows solved on a background thread), `margin` (`hj` / `hj_backup` distance margin in m; < 0: the filter's default);
- `map_origin` / `map_size` (default: the profile's map);
- `log_path` (CSV per period: t, pose, w_nom, w, decision time, intervened, interventions, corner windows in use, corner solves and their time).

**Frames.** Gazebo's diff-drive odometry starts at the spawn pose. The launch file therefore passes `x_pose` / `y_pose` / `yaw_pose` to the node, which maps odometry to world coordinates. The map, the goal and the profile are in world coordinates. The map and markers are published in the `odom` frame.

**RViz topics.**
- `map` (`nav_msgs/OccupancyGrid`);
- `fastbrt_markers`: obstacle faces (blue), filter region (purple), convex corners (red), concave corners (orange);
- `obstacle_events` (`std_msgs/String`): one JSON object per detection event: stable id, kind, faces with lengths, corners, in world coordinates.

**To check on the simulation machine:**
- `/scan` and `/odom` are published;
- the robot moves on `/cmd_vel` (`Twist`, per the bridge parameters);
- the decision time stays below `ctrl_dt` (see `log_path`).

## TurtleBot3

### Setup

Follow the setup instructions here: https://docs.robotis.com/docs/systems/turtlebot3/simulation/gazebo_simulation

```
cd turtlebot3
colcon build --packages-select turtlebot3_controls
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_controls hj_launch_world.launch.py world:=turtlebot3_controls/hj_reach_target.world x_pose:=-0.8 y_pose:=-0.8 yaw_pose:=0.2
```

On Humble use `hj_launch_world_classic.launch.py` with the same arguments (Gazebo Classic; the `gazebo_ros` plugins publish `/scan` and `/odom` and subscribe to `/cmd_vel` directly, no bridge). On the new Gazebo, the bridge (`turtlebot3_controls/params/turtlebot3_burger_bridge.yaml`) forwards `/scan`, `/odom`, `/tf`, `/imu` and `cmd_vel`. The Burger's LDS-01 LiDAR has 360 beams, a 0.12–3.5 m range and a 5 Hz rate. The Burger profile cuts the range at 0.8 m in software, so that obstacles are discovered while driving.

### Launch simulation

We are following the basic example from https://docs.robotis.com/docs/systems/turtlebot3/simulation/gazebo_simulation.

```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_controls hj_launch_world.launch.py world:=turtlebot3_controls/hj_reach_target.world x_pose:=-0.8 y_pose:=-0.8 yaw_pose:=0.2
```

In `turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/params/turtlebot3_waffle_bridge.yaml` (or the burger equivalent), change `TwistStamped` to `Twist`:

```
- ros_topic_name: "cmd_vel"
  gz_topic_name: "cmd_vel"
  ros_type_name: "geometry_msgs/msg/TwistStamped"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
```

### Legacy: offline HJ with a known map, open-loop replay

This is the previous pipeline, kept as a baseline.

1. `reach_avoid.py` computes the reach-avoid HJ value with `optimized_dp` from the **known** obstacles. It then simulates the 2D Dubins car (equivalent to the TurtleBot) in the solver's own model, and writes the turn rates to `dubins_control.txt`. Follow the `optimized_dp` setup steps and set up its conda environment first:

   ```
   cd turtlebot3/turtlebot3_controls/turtlebot3_controls/script
   conda activate odp
   python3 reach_avoid.py
   ```

2. `controls_keyboard.py` then replays these commands at 10 Hz, without reading `/odom`:

   ```
   cd turtlebot3
   export TURTLEBOT3_MODEL=burger
   colcon build --packages-select turtlebot3_controls
   source install/setup.bash
   ros2 run turtlebot3_controls controls_keyboard
   ```

Known mismatches in this baseline:
- The solver steps at 0.05 s, but the replay runs at 0.1 s.
- The replay publishes a linear velocity of 1.0 m/s. The solver assumes 0.5 m/s, and the Burger's maximum is about 0.22 m/s.
- The replay is open loop, with no state feedback.
- The arena walls are not obstacles in the solve.

### Troubleshooting

If the simulation has been run several times in one sitting, old Gazebo runs may still be running in the background. Make sure they are all gone:

```
pkill -f gazebo
pkill -f ros
```

## Design decisions

Decisions taken so far, with their reasons, so that they are not revisited by accident. Decisions 1–5 date from 2026-11-15; decision 3 was revised and decisions 6–9 were added on 2026-11-21. The full list is in [docs/11 §11.2](docs/11_roadmap.md).

**1. Two layers: region for safety, obstacles for primitives (hybrid).** Both layers are computed from the accumulated occupancy grid in every control period.
- *Safety layer (region).* The region is the part of the map the robot may use, shrunk by the footprint: an outer boundary plus one closed ring around every obstacle inside it. The complement of the region is the modelled obstacle. Witnesses are verified against its signed distance. This layer guarantees that every detected cell is covered and handles frontiers and occlusion.
- *Obstacle layer.*
  - Connected components of occupied cells are the obstacles.
  - Each obstacle yields faces: enclosing lines along its free-facing boundary, with collinear pieces merged.
  - Faces give side-length estimates, obstacle IDs and the list of corners where two faces meet.
  - The faces (plus frontier edges, in the `unknown=occupied` mode) become the primitives. The primitives feed the HJ terms: candidate controls, U.
- *Why not region only.* There is no obstacle identity, no side lengths and no corner list, and edges are cut at arbitrary points. It also does not match the method's story: detect an obstacle → primitive → BRT.
- *Why not obstacles only.* Coverage, occlusion and unknown space would have to be redesigned and re-proven.
- *Consequences.* Safety never depends on how well the faces are fitted; fitting quality affects only efficiency (interventions, timeouts). Taking obstacles from the accumulated map, not from single scans, avoids data association across scans: an obstacle grows in the map as more of it is seen.

**2. Obstacles that touch are one obstacle.** A connected component of occupied cells is one obstacle, even when it is physically several objects. For example, in `hj_reach_target.world` obstacle 1, obstacle 2 and the west wall touch, so they form a single obstacle. The faces and corners are still correct; only the IDs merge. Splitting such a component (e.g. at concave corners) is possible later if needed.

**3. Inward placement, one primitive per face (revised 2026-11-21).** A primitive of length L is an L × L/√α rectangle.
- *New rule.* The rectangle's free-facing side lies on the face, so all of its thickness is inside the obstacle. The BRT on the free side then depends only on L, and long faces need no cutting.
- *Faces shorter than L_min.* They are extended symmetrically to `L_min ≈ 2·dx·√α` (about 0.45 turning radii for a 241 grid and α = 20). This is conservative.
- *The old rule was a deployment deviation.* The face was the rectangle's midline, and long faces were cut into collinear pieces so that a long wall's primitive would not reach far into free space.
- *Code.* It is `placement="inward", cut=False`. The default still uses the old rule until the offline `V₀` is loaded.

**4. Corner list for the leaking-corner update.** The obstacle layer outputs where two faces of an obstacle meet at an angle. This list is the input for the leaking-corner local update (HJ_local_update, under fast-brt). The exact format is to be agreed with the HJ_local_update side.

**5. Two filter modes are kept for comparison.**
- `unknown=occupied` treats unknown space as an obstacle. It is safe against occlusion surprises.
- `unknown=free` counts only detected obstacles.

**6. Primitives use the BRT, not the BRS.** The BRS (`TargetSetMode "None"`) has height 0 in front of a face: a state heading straight into the face is judged safe. Avoidance needs the BRT (`minVWithV0`). The local update has a BRT option too (`brt=True` in HJ_local_update, `target_set_mode="brt"` in fast-brt).

**7. Offline value: linear encoding, first order, 241 grid, α = 20.**
- *Encoding.* The value is `max(|x|, √α|y|) − L/2`, odp `accuracy="low"`, solved on a 241 grid (`v0_dubins_3d_linear_alpha20_low.npz`).
- *Why first order.* The local update recomputes first-order steps, so every primitive that goes into it must be solved with `accuracy="low"`; using one V₀ everywhere keeps it simple.
- *Accuracy.* The old setup was a quadratic encoding, first order, 121 grid. It underestimates the BRT in front of a face by about 0.22 turning radii on average. The new setup is off by about 0.09; `accuracy="medium"` would give about 0.04 but cannot be used with the local update ([docs/10](docs/10_primitive_alpha_accuracy.md)).
- *Options.* Linear / quadratic × low / medium V₀ are all kept, to compare them (`experiments/filter_batch.py --v0 ...`).

**8. Three filter modes are kept as options.**
- `hj`: the classic HJ filter on the primitive values (`HJFilter`): nominal while the distance margin of U = min_i V_i (V / √α, in metres, the same for both encodings) stays above `margin`, else the HJ evasion. Not safe at leaking corners. `robust=True` adds the map clearance and the sampling gap between sub-steps.
- `backup`: the backup filter (`BackupFilter`). It accepts the nominal control only if a safe backup maneuver exists afterwards.
- `hj_backup`: the HJ filter proposes, the backup filter checks (`HJBackupFilter`). Its `margin` must be positive (default 2 cm), otherwise the backup filter always acts first.
- Leaking corners: `corners=True` (any filter) corrects the union near the robot's corners with a stationary local update on a small window (fast-brt `fast_brt/corner.py`, [docs/13](docs/13_online_local_update.md)); `corner_mode` `sync`, `delayed` (simulation, with the solve latency) or `thread` (online). With it, `hj` collides far less at corners (`hj_reach_target`, robot radius 0.124 m: 7 → 2 collisions, the corrected value is off by 5–12 mm there; with 0.105 m it was 7 → 0). `hj_backup` and `backup` have no collision in any batch.

**9. Dynamics order.** Constant-speed Dubins 3D comes first; its parameters will be swept later. Dubins 4D (QBot) follows, and a 6D drone after that.

## TODO

**Integration**
- [x] Add fast-brt as a submodule (`fast-brt/`, branch `robot-sim`), with HJ_local_update as its submodule (branch `robot-sim`); a recursive clone fetches all three levels
- [ ] Pin the `optimized_dp` version fast-brt needs: the old `Grid(min, max, dims, pts, pDim)` signature, from before odp commit 689548e
- [x] Confirm the ROS 2 distro and Gazebo version on the simulation machine: Ubuntu 22.04, Humble, Gazebo Classic 11 (`hj_launch_world_classic.launch.py`)
- [ ] First run of `fastbrt_nav` in Gazebo Classic on the desktop ([docs/14](docs/14_ros2_desktop_test.md))
- [ ] Register `optimized_dp` as a submodule. It is listed in `.gitmodules` but is not a gitlink in the tree, so a recursive clone does not fetch it.
- [ ] Decide whether both `unitree_mujoco/` and `mujoco/unitree_mujoco/` are needed (they pin the same commit).

**Navigation (this repository)**
- [x] Robot profiles and units (`robots/`, `nav_core.robots`)
- [x] Accumulated occupancy grid and boundary edges with obstacle side-length estimates (`nav_core.mapping`)
- [x] Pure-pursuit nominal controller for Dubins 3D (`nav_core.nominal`)
- [x] ROS-free closed-loop simulator on the Gazebo SDF world (`nav_core.sim`, `scripts/run_sim.py`)
- [x] Backup filter (formerly witness gatekeeper) as a `nav_core.sim` safety hook (Dubins 3D), with free-standing obstacles and two modes (unknown occupied / free)
- [x] Inward primitive placement option (`placement="inward"`) and the α study (`experiments/alpha_study.py`, [docs/10](docs/10_primitive_alpha_accuracy.md))
- [x] Backup filter ported into `nav_core/safety` as `BackupFilter`, using fast-brt's point query and the offline V₀; defaults inward placement, no cutting ([roadmap](docs/11_roadmap.md) step 4)
- [ ] Rerun the batches with the new V₀ and update the docs (step 5)
- [x] Filter modes `hj` and `hj_backup` (`nav_core.safety.HJFilter`, `HJBackupFilter`, `make_filter`)
- [x] Occluded test world (`worlds/occluded_4x4.world`) and LiDAR-range sweep of the two modes
- [ ] Start-up check that the LiDAR range lets `unknown=occupied` start (≈ robot radius + 2 × turning radius + margin)
- [x] Obstacle layer (Design decisions 1–4, `nav_core.mapping.extract_obstacles`): connected components → enclosing faces (shallow turns merged) → lengths, IDs, corners (convex / concave)
- [x] Backup filter primitives from the obstacle layer (`primitives="faces"`, default) + frontier edges; margin still from the region
- [ ] Derive the face-fit tolerance from the sensor noise (profile field). Defaults to 1.5 cells, which suits ~1 cm noise with several scans per place; 2 cm noise needs about 2 cells
- [x] Stable obstacle ids and detection events (`ObstacleTracker`; `obstacle_events` topic in the ROS node)
- [ ] Agree on the corner-list / event format with the HJ_local_update side
- [ ] Recompute primitives only for obstacles with an event (now every period, from scratch)
- [x] Edge-cell flicker under grazing rays: counting map model (τ = 0.2) + graze protection; flicker 38 / 80 / 112 → 2 / 15 / 27 events
- [x] Map-aware MPPI nominal controller (`nav_core.nominal.MPPI`: geodesic cost-to-go on the accumulated map + obstacle distance)
- [x] ROS 2 node `fastbrt_nav` (written; conversions unit-tested; not yet run in Gazebo):
  - subscribe to `/scan` and `/odom`, reading the scan geometry from the message;
  - run mapping → nominal → filter and publish `cmd_vel`;
  - publish RViz markers (grid, edges) and log trajectories.
- [ ] Closed-loop Burger in Gazebo with the nominal controller only (no filter), to check units, speed limits and timing
- [ ] Our own copy of the Burger SDF, with the LiDAR rate and range taken from the profile
- [ ] Generic MPPI nominal controller on fast-brt dynamics (3D first, then 4D / 6D)
- [ ] QBot: Gazebo model and profile values
- [ ] Crazyflie: 6D model, simulator, sensor and profile values

**Safety (fast-brt, branch `robot-sim`)**
- [x] Import `fast_brt` without HeteroCL (lazy import of HJ_local_update)
- [x] Offline accuracy study (`experiments/primitive_accuracy.py`, run on mars-14)
- [x] BRT option for the local update (`target_set_mode="brt"`; HJ_local_update `brt=True`, numpy and HeteroCL). The numpy version matches a direct BRT solve on an L shape in sign.
- [x] numpy local update option (`local_update_impl="numpy"`)
- [ ] Check the HeteroCL BRT local update and its timing on mars-14 (`pytest -m slow -k tube`, `experiments/local_update_timing.py`)
- [ ] Export the offline `V₀` (BRT, chosen accuracy, final slice + grid parameters) from a CLI (roadmap step 1), then run it on mars-14 (step 3)
- [ ] numpy point query without odp: length shift, inward placement, short-face extension, `V`, `∂V/∂θ`, `u*` (step 2)
- [ ] Second-order local update (the kernel is first order now), or a hybrid correction
- [ ] Online local update: a window that moves with the robot; incremental updates per obstacle id
- [ ] Dubins 4D and 6D dynamics and their offline values
- [ ] A safety-filter interface that takes the nominal control as input, e.g. `SafetyFilter.step(x, u_nom, scene)` (in `nav_core/safety`)

## Open questions

- **ROS 2 distro and Gazebo version.** Answered: the desktop runs Ubuntu 22.04 with Humble, so Gazebo Classic 11 with the apt `ros-humble-turtlebot3-gazebo`. The pinned `turtlebot3_simulations` commit (`main@9be186f`, `gz_sim_vendor`) is for Jazzy and is not built there; `hj_launch_world_classic.launch.py` loads the same worlds in Classic.
- **HJ_local_update online.** The HeteroCL kernel needs the `odp` conda environment, but online code runs in the ROS 2 Python. A numpy implementation exists. Is it fast enough, or should the update run as a separate service, triggered by detection events?
- **Crazyflie sensor and simulator.** Which simulator? Which sensor: an ideal LiDAR, or the 5-beam Multi-ranger deck?
