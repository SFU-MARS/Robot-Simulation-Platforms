# Robot Simulation Platforms

This repository follows the [Fast-BRT Experiment Document](https://docs.google.com/document/d/1S5lzXlQSW35xeCZrHe30NN7NRfxiYE0QOhNOWc51FYE/edit?tab=t.0). Most folders are git submodules. Clone them explicitly by adding `--recurse-submodules` to `git clone`, or run `git submodule update --init --recursive` after a checkout.

**[`docs/`](docs/) walks through the code module by module (in Chinese), ending with one full MPPI control period from loading the map to the 2D primitives.** This README stays the quick reference: setup, usage, results and TODO.

## Goal

The robot navigates an **unknown** map: it does not know the obstacles in advance. It senses them with a LiDAR, builds its own map while driving, and reaches a goal. An HJ reachability safety filter keeps it safe on the way.

Each control period:

```
/scan + /odom ──► mapping: accumulated occupancy grid ──► boundary edges (obstacle | frontier)
                                                                │  edge length = obstacle side estimate
/odom ──► nominal controller (obstacle-unaware) ──► u_nom       ▼
                                                  HJ safety filter (FaST-BRT) ──► /cmd_vel
```

- **Nominal controller.** It drives towards the goal and carries no safety guarantee. Two are available: pure pursuit, which ignores the map, and MPPI on the accumulated map, which is optimistic about unknown space.
- **Safety filter.** There is one HJ primitive per boundary edge, transported from a single precomputed value function. The filter overrides the nominal control only when needed. Unknown space is treated as occupied.
- **Map.** It accumulates over the whole run. The environment is static, so the known free space only grows. In simulation the pose is known (odometry / ground truth), so no SLAM is needed.

### Division of work

| | [FaST-BRT](https://github.com/Hu-Hanyang/FaST-BRT) (submodule, planned) | this repository |
| --- | --- | --- |
| Scope | dynamics, HJ value functions, primitives, safety filter, leaking-corner updates ([HJ_local_update](https://github.com/SFU-MARS/HJ_local_update)) | navigation: robot profiles, mapping, nominal controllers, ROS 2 / Gazebo integration |
| Depends on | numpy / scipy; `optimized_dp` offline only | FaST-BRT, ROS 2 |

- **Dependency direction.** Dependencies point one way: this repository imports FaST-BRT, never the reverse. No HJ code lives here.
- **No online solves.** Values are precomputed offline with `optimized_dp` and loaded at start-up.
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
  robots/                       # per-robot profiles: burger.yaml, qbot.yaml, crazyflie.yaml
  nav_core/                     # ROS-free navigation core (pip-installable project)
    pyproject.toml
    nav_core/                   # the importable package
      robots/                   #   profile loading, physical <-> model units
      mapping/                  #   occupancy grid, region, boundary edges, obstacle layer (faces, corners)
      nominal/                  #   nominal controllers: pursuit, map-aware MPPI
      sim/                      #   ROS-free closed loop: SDF world, synthetic LiDAR, unicycle plant
      safety/                   #   FaST-BRT filters as safety hooks (unit conversion, map -> scene)
    scripts/run_sim.py
    tests/
  turtlebot3/
    turtlebot3_controls/        # ROS 2 package: worlds (hj_reach_target, worlds/occluded_4x4), launch, bridge params, legacy scripts
    turtlebot3_simulations/     # submodule (ROBOTIS), not modified
  ros2/fastbrt_nav/             # ROS 2 node: /scan + /odom -> nav_core + FaST-BRT -> /cmd_vel, RViz map and markers
  mujoco/unitree_mujoco/        # submodule (Unitree, MuJoCo)
  unitree_mujoco/               # submodule (same commit as above)
```

Planned additions:
- `FaST-BRT/`, as a submodule;
- our own copy of the robot SDF models, so that the LiDAR range and rate can be set per profile without modifying `turtlebot3_simulations`.

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

**ROS-free closed loop.** `nav_core.sim` stands in for one Gazebo run. It reads the collision boxes of the same SDF world, keeping those that span the LiDAR plane. It ray-casts a synthetic scan that follows the `LaserScan` conventions and integrates a unicycle exactly. Every control period it runs scan → map → nominal → optional safety hook `safety(x, w_nom, grid) -> w`. The FaST-BRT filter plugs into this hook.

```
cd nav_core
PYTHONPATH=. python scripts/run_sim.py                   # burger, hj_reach_target.world, (-0.8, -0.8, 0.2) -> (0.7, 0.2)
PYTHONPATH=. python scripts/run_sim.py --noise 0.01      # 1 cm range noise
```

With the nominal controller only, the Burger grazes the corner of obstacle 2 and collides at t ≈ 7.3 s. This is the case the safety filter has to handle. Figures are written to `nav_core/outputs/`.

**FaST-BRT witness gatekeeper (prototype).** `nav_core.safety.FastBrtGatekeeper` plugs FaST-BRT's gatekeeper into the safety hook. Each period it does three things:
1. It takes the region the robot may use from the accumulated map, shrunk by the footprint (`free_region`). The region is an outer boundary plus one closed ring around every obstacle inside it.
2. It puts one primitive on every ring edge.
3. It accepts the nominal turn rate only if a witness exists from the next state: a rollout that keeps the clearance and ends on a free loiter circle.

Two modes (`unknown=`), to be compared:
- `occupied` (default): unknown space is an obstacle. The region is the known free space, so frontier edges are obstacles too. This mode is safe even against obstacles hidden behind others.
- `free`: only detected obstacles count. The region is everything but the detected obstacles. It is less conservative, but it is safe only if every obstacle is seen before the robot enters its BRT.

In both modes the modelled obstacle is the complement of the region. Every detected obstacle cell, grown by the footprint, is therefore covered completely, not just outlined by thin primitives. A free-standing obstacle (a pillar, a rock) is enclosed by its own ring. This holds even when only one face has been seen. `RegionScene` generalizes FaST-BRT's `SceneModel` (one simple polygon) to these regions. It belongs upstream in FaST-BRT. Until FaST-BRT is a submodule, `fast_brt` is imported from `PYTHONPATH`:

```
PYTHONPATH=.:<path to FaST-BRT> python scripts/run_sim.py --filter gatekeeper [--unknown free]
```

Burger, default start and goal: the run reaches the goal at t ≈ 17.6 s, with 7 interventions in 88 periods and a 5.5 cm minimum clearance. Batches of random starts × goals gave these results (no collision with either filter mode):

| world | | reached | collision | timeout |
| --- | --- | --- | --- | --- |
| `hj_reach_target` (26 runs) | nominal only | 10 | 16 | 0 |
| | unknown occupied | 13 | 0 | 13 |
| | unknown free | 13 | 0 | 13 |
| walls + free-standing pillar (35 runs) | nominal only | 22 | 13 | 0 |
| | unknown occupied | 35 | 0 | 0 |
| | unknown free | 35 | 0 | 0 |

- **Modes look alike here.** The two modes barely differ because the 2 m arena with a 0.8 m LiDAR range has little occlusion. A larger, more occluded world is needed to compare them.
- **Timeouts.** Pure pursuit ignores obstacles, so the gatekeeper keeps rejecting it and the robot loiters on its certified circle. This is a liveness problem of the nominal controller, not of the filter.
- **Geometry.** The 2 m arena is tight for a certified loiter circle. What matters is the turning radius v / ω_max. At 0.2 m there is no witness at the start, so the Burger profile uses 0.1 m (v = 0.1 m/s, ω_max = 1 rad/s). This is also within the differential-drive limit v + ω_max · track / 2 ≤ wheel speed, which `load_profile` checks. The Burger's 0.22 m/s and 2.84 rad/s maxima cannot be reached at the same time.
- **Edge cutting.** Edges are cut so that no primitive reaches more than one cell off its edge (`h_max`).

**Nominal controllers.**
- `pursuit` always turns the robot's heading towards the goal, at a turn rate proportional to the heading error (capped at ω_max). It ignores the map, so it drives straight into obstacles.
- `mppi` samples many short turn-rate sequences, simulates each one, and blends them, favouring low cost. The cost has two parts:
  - the shortest-path distance to the goal through the known map, computed around obstacles;
  - a penalty for getting close to occupied cells.

  It is optimistic: unknown space counts as drivable.

Batches of random starts × goals, with no filter and with the gatekeeper:

| world | nominal | nominal only | + gatekeeper |
| --- | --- | --- | --- |
| `hj_reach_target` (26 runs) | pursuit | 10 reached, 16 collisions | 13 reached, 0 collisions, 13 timeouts |
| | mppi | 19 reached, 7 collisions | 13 reached, 0 collisions, 13 timeouts |
| walls + pillar (35 runs) | pursuit | 22 reached, 13 collisions | 35 reached |
| | mppi | 34 reached, 1 collision | 35 reached, filter never had to intervene |

**Why the 13 timeouts in `hj_reach_target` do not go away with MPPI.** The robot model cannot stop: it always drives at constant speed and can only steer. The only way to "wait safely" is to drive in a tight circle (radius 0.1 m). The gatekeeper therefore accepts a command only if, afterwards, the robot still has a way to reach known free space wide enough to circle in. That takes about 0.45 m: the circle's 0.2 m diameter, plus the robot's width (2 × 0.105 m), plus a margin. The upper-left area of this world is reachable only through a 0.4 m gap between obstacle 1 and the north wall. The robot itself would fit, but it could not circle inside the gap, so the gatekeeper never lets it in. Exactly 13 of the 26 runs need that gap. This is a limit of the constant-speed model, not of the nominal controller. A robot that can brake (Dubins 4D, QBot) can "wait safely" by stopping and does not have it.

**Occluded world and the two filter modes.** `worlds/occluded_4x4.world` is a 4 × 4 m arena with:
- three screens, each with a box hidden right behind one of its ends;
- three free-standing pillars.

The sweep uses MPPI as the nominal controller and varies the LiDAR range. There are 26 runs per row: random starts × goals plus the three traps.

| LiDAR range | no filter | unknown = occupied | unknown = free |
| --- | --- | --- | --- |
| 0.3 m (3 × turning radius) | 22 reached, 4 collisions | cannot start (26 runs) | 21 reached, **5 collisions** |
| 0.5 m (5 ×) | 24 reached, 2 collisions | 19 reached, 0 collisions, 7 timeouts | 20 reached, 0 collisions, 6 timeouts |
| 0.8 m (8 ×) | 22 reached, 4 collisions | 20 reached, 0 collisions, 6 timeouts | 20 reached, 0 collisions, 6 timeouts |

How to read the table:
- *`unknown = free` is not safe when the LiDAR range is short.* This mode treats unseen space as drivable, so the filter can approve a path into space that hides an obstacle. With a 0.3 m range, the obstacle is seen too late to turn away.
- *`unknown = occupied` never collides, but needs a minimum range.* After the first scan, the known free space must already hold the circle the robot waits on: roughly LiDAR range ≥ robot radius + 2 × turning radius + margin ≈ 0.34 m. At 0.3 m it cannot start.
- *At 5× and 8× the turning radius both modes are safe.* `free` was not faster.

**Obstacle layer.** `extract_obstacles(grid, offset=0)` returns the obstacles of the accumulated map (README, Design decisions 1–4):
- the obstacles themselves: connected components of occupied cells, with touching objects merged into one;
- their faces: enclosing lines with a free-side normal;
- their corners: where two faces meet, flagged convex or concave with respect to the obstacle.

Results:
- *Test world, noise-free.* `hj_reach_target` gives one obstacle with 12 faces. Its 4 convex corners are exactly the outer corners of obstacles 1 and 2. Obstacle 1's sides come out as 0.43 / 0.50 / 0.53 / 0.39 m (true 0.4 × 0.5 m).
- *Enclosure.* Every face lies at least 0.95 cm outside the true obstacles. With `offset` = the footprint radius, the faces describe the configuration-space obstacle.
- *Noise.* With 1 cm range noise and 10 scans per place, there are 13 faces and 5 convex corners. Here a face can cut up to 0.6 cm into the true obstacle, because the map itself is noisy; the safety clearance (1.5 cells) covers this.

Gatekeeper comparison, primitives from faces vs from the region's rings: there are no collisions with either. Reached and timeout counts are identical. The minimum clearance grows from 3.4 to 5.5 cm (`hj_reach_target`) and from 2.5 to 4.8 cm (pillar world). This is because the faces enclose the obstacles with square corners. The cost stays at about 15–30 ms per step.

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
- `FastBrtGatekeeper` gives its faces stable ids and keeps the period's events in `gate.events`.
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

Switching the default did not change any batch result above except the occluded sweep, which moved by one or two runs (the numbers shown are with the new model). There were no collisions with either filter mode.

**Scan conventions** (REP 117):
- `+inf`, or a reading ≥ range max, means no return: the ray is free up to the range.
- `nan`, `-inf` and readings below `range_min` are invalid and are skipped. They never clear cells.

**Units.** FaST-BRT's Dubins 3D model is nondimensional (v = 1, |ω| ≤ 1). For the Burger profile (v = 0.1 m/s, ω_max = 1 rad/s), one model length unit is 0.1 m and one model time unit is 1 s (`profile.units`).

## ROS 2 node: fastbrt_nav

`ros2/fastbrt_nav` runs the same loop as `nav_core.sim` against Gazebo (or a real robot):

```
/scan, /odom → accumulated map → nominal (pursuit | mppi) → FaST-BRT gatekeeper → /cmd_vel
```

Behaviour:
- It integrates the newest scan at the newest pose, once per control period (the profile's `ctrl_dt`).
- It publishes the gatekeeper's zero-order-hold turn rates on a sub-period timer.
- It drives at the profile's constant speed. It stops (zero `cmd_vel`) before the first scan, at the goal, and if the filter cannot certify the start or fails.
- It uses only `LaserScan`, `Odometry` and `Twist`, so the same code runs on Humble and Jazzy.

It has not run in Gazebo yet. Only the message conversions (`fastbrt_nav/conversions.py`) are unit-tested, without ROS.

Build and run (from `turtlebot3/`, next to `turtlebot3_controls`):

```
export PYTHONPATH=<repo>/nav_core:<path to FaST-BRT>:$PYTHONPATH     # nav_core and fast_brt are not ROS packages
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
- `nominal` (`mppi` | `pursuit`), `filter` (`gatekeeper` | `none`), `unknown` (`occupied` | `free`), `primitives` (`faces` | `rings`);
- `map_origin` / `map_size` (default: the profile's map);
- `log_path` (CSV of t, pose, w_nom, w and decision time per period).

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

The bridge (`turtlebot3_controls/params/turtlebot3_burger_bridge.yaml`) already forwards `/scan`, `/odom`, `/tf`, `/imu` and `cmd_vel`. The Burger's LDS-01 LiDAR has 360 beams, a 0.12–3.5 m range and a 5 Hz rate. The Burger profile cuts the range at 0.8 m in software, so that obstacles are discovered while driving.

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

Decisions taken so far, with their reasons, so that they are not revisited by accident. Date: 2026-11-15.

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

**3. Long faces are cut into several collinear primitives.** A primitive of length L is an L × L/√α rectangle, so one primitive per long wall would reach far into the free space. A face keeps one ID and one length estimate. The cut points are collinear, so they are not corners.

**4. Corner list for the leaking-corner update.** The obstacle layer outputs where two faces of an obstacle meet at an angle. This list is the input for the leaking-corner value update (HJ_local_update, in FaST-BRT). The exact format is to be agreed with the HJ_local_update side.

**5. Two filter modes are kept for comparison.**
- `unknown=occupied` treats unknown space as an obstacle. It is safe against occlusion surprises.
- `unknown=free` counts only detected obstacles.

## TODO

**Integration**
- [ ] Add FaST-BRT as a submodule pinned to a commit, after the leaking-corner work is pushed, and agree on one integration branch.
- [ ] Confirm the ROS 2 distro and Gazebo version on the simulation machine (`echo $ROS_DISTRO`, `lsb_release -rs`, `gz sim --version`); see Open questions.
- [ ] Register `optimized_dp` as a submodule. It is listed in `.gitmodules` but is not a gitlink in the tree, so a recursive clone does not fetch it.
- [ ] Decide whether both `unitree_mujoco/` and `mujoco/unitree_mujoco/` are needed (they pin the same commit).

**Navigation (this repository)**
- [x] Robot profiles and units (`robots/`, `nav_core.robots`)
- [x] Accumulated occupancy grid and boundary edges with obstacle side-length estimates (`nav_core.mapping`)
- [x] Pure-pursuit nominal controller for Dubins 3D (`nav_core.nominal`)
- [x] ROS-free closed-loop simulator on the Gazebo SDF world (`nav_core.sim`, `scripts/run_sim.py`)
- [x] FaST-BRT witness gatekeeper as a `nav_core.sim` safety hook (prototype: `fast_brt` from `PYTHONPATH`, Dubins 3D), with free-standing obstacles and two modes (unknown occupied / free)
- [x] Occluded test world (`worlds/occluded_4x4.world`) and LiDAR-range sweep of the two modes
- [ ] Start-up check that the LiDAR range lets `unknown=occupied` start (≈ robot radius + 2 × turning radius + margin)
- [x] Obstacle layer (Design decisions 1–4, `nav_core.mapping.extract_obstacles`): connected components → enclosing faces (shallow turns merged) → lengths, IDs, corners (convex / concave)
- [x] Gatekeeper primitives from the obstacle layer (`primitives="faces"`, default) + frontier edges; margin still from the region
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
- [ ] Generic MPPI nominal controller on FaST-BRT dynamics (3D first, then 4D / 6D)
- [ ] QBot: Gazebo model and profile values
- [ ] Crazyflie: 6D model, simulator, sensor and profile values

**Safety (FaST-BRT)**
- [ ] Dubins 4D and 6D dynamics and their offline values
- [ ] Leaking-corner value updates (HJ_local_update) inside FaST-BRT
- [ ] A safety-filter interface that takes the nominal control as input, e.g. `SafetyFilter.step(x, u_nom, scene)`
- [ ] Move `RegionScene` (`nav_core/safety/fastbrt_gatekeeper.py`) into FaST-BRT. It generalizes `SceneModel` from one simple polygon to an outer boundary plus closed rings around obstacles, which accumulated maps with free-standing obstacles need.
- [ ] Save / load of the precomputed values together with their grid parameters

## Open questions

- **ROS 2 distro and Gazebo version.** The plan is Humble. However:
  - the pinned `turtlebot3_simulations` commit (`main@9be186f`) depends on `gz_sim_vendor`, which exists only for Jazzy;
  - Humble's apt `turtlebot3_gazebo` targets Gazebo Classic.

  Which distro and which Gazebo does the simulation machine actually run? New code targets Python 3.10, so it runs on both.
- **HJ_local_update.** Does it run online, and if so, does it need HeteroCL? Online code runs in the ROS 2 Python, not in the `odp` conda environment.
- **Crazyflie sensor and simulator.** Which simulator? Which sensor: an ideal LiDAR, or the 5-beam Multi-ranger deck?
