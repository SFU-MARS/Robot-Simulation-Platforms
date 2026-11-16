# Robot Simulation Platforms

This repository follows the [Fast-BRT Experiment Document](https://docs.google.com/document/d/1S5lzXlQSW35xeCZrHe30NN7NRfxiYE0QOhNOWc51FYE/edit?tab=t.0). Most folders are git submodules. Clone them explicitly by adding `--recurse-submodules` to `git clone`, or run `git submodule update --init --recursive` after a checkout.

## Goal

The robot navigates an **unknown** map: it does not know the obstacles in advance. It senses them with a LiDAR, builds its own map while driving, and reaches a goal. An HJ reachability safety filter keeps it safe on the way.

Each control period:

```
/scan + /odom ──► mapping: accumulated occupancy grid ──► boundary edges (obstacle | frontier)
                                                                │  edge length = obstacle side estimate
/odom ──► nominal controller (obstacle-unaware) ──► u_nom       ▼
                                                  HJ safety filter (FaST-BRT) ──► /cmd_vel
```

- **Nominal controller.** It drives towards the goal, ignores obstacles and carries no safety guarantee. Pure pursuit comes first, MPPI later.
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
      mapping/                  #   occupancy grid, boundary edges (obstacle / frontier)
      nominal/                  #   nominal controllers (pursuit; MPPI planned)
    tests/
  turtlebot3/
    turtlebot3_controls/        # ROS 2 package: world, launch, bridge params, legacy scripts
    turtlebot3_simulations/     # submodule (ROBOTIS), not modified
  mujoco/unitree_mujoco/        # submodule (Unitree, MuJoCo)
  unitree_mujoco/               # submodule (same commit as above)
```

Planned additions:
- `FaST-BRT/`, as a submodule;
- a ROS 2 node package `fastbrt_nav`: scan / odom → `nav_core` + FaST-BRT → `cmd_vel`;
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

**Units.** FaST-BRT's Dubins 3D model is nondimensional (v = 1, |ω| ≤ 1). For the Burger profile (v = 0.2 m/s, ω_max = 1 rad/s), one model length unit is 0.2 m and one model time unit is 1 s (`profile.units`).

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
- [ ] ROS 2 node `fastbrt_nav`:
  - subscribe to `/scan` and `/odom`, reading the scan geometry from the message;
  - run mapping → nominal → filter and publish `cmd_vel`;
  - publish RViz markers (grid, edges) and log trajectories.
- [ ] Closed-loop Burger in Gazebo with the nominal controller only (no filter), to check units, speed limits and timing
- [ ] Our own copy of the Burger SDF, with the LiDAR rate and range taken from the profile
- [ ] Generic MPPI nominal controller on FaST-BRT dynamics (3D first, then 4D / 6D)
- [ ] Start-up check that the sensing range covers the BRT reach of the robot's dynamics
- [ ] QBot: Gazebo model and profile values
- [ ] Crazyflie: 6D model, simulator, sensor and profile values

**Safety (FaST-BRT)**
- [ ] Dubins 4D and 6D dynamics and their offline values
- [ ] Leaking-corner value updates (HJ_local_update) inside FaST-BRT
- [ ] A safety-filter interface that takes the nominal control as input, e.g. `SafetyFilter.step(x, u_nom, scene)`
- [ ] A scene model built from an edge set + signed distance, for accumulated maps whose free space has holes
- [ ] Save / load of the precomputed values together with their grid parameters

## Open questions

- **ROS 2 distro and Gazebo version.** The plan is Humble. However:
  - the pinned `turtlebot3_simulations` commit (`main@9be186f`) depends on `gz_sim_vendor`, which exists only for Jazzy;
  - Humble's apt `turtlebot3_gazebo` targets Gazebo Classic.

  Which distro and which Gazebo does the simulation machine actually run? New code targets Python 3.10, so it runs on both.
- **HJ_local_update.** Does it run online, and if so, does it need HeteroCL? Online code runs in the ROS 2 Python, not in the `odp` conda environment.
- **Crazyflie sensor and simulator.** Which simulator? Which sensor: an ideal LiDAR, or the 5-beam Multi-ranger deck?
