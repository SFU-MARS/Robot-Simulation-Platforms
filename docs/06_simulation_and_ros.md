# 06 仿真与 ROS 2 节点

## 6.1 为什么要一个不依赖 ROS 的仿真

[`nav_core/sim/`](../nav_core/nav_core/sim/) 是 Gazebo 的替身，但不是玩具：

- **场景直接读 Gazebo 的 SDF world 文件**，障碍和 Gazebo 里完全一致；
- 合成 lidar 遵循 `LaserScan` 的约定（`+inf`、`nan`、`range_min`）；
- plant 用 unicycle 精确圆弧积分。

好处：在没有 ROS 的 laptop 上就能跑完整的闭环和 batch 实验（几十次 episode 几分钟），上 Gazebo 之前就能把参数、单位、算法全部验证完。

## 6.2 World：从 SDF 读障碍

代码：[`sim/world.py`](../nav_core/nav_core/sim/world.py)

```python
world = World.from_sdf(path, plane_z=profile.sensor.height)
```

遍历 SDF 里所有 `<model>/<link>/<collision>`，组合位姿，**只保留跨越扫描平面 `plane_z` 的 box**。`hj_reach_target.world` 有 14 个 model，在 0.17 m 的扫描高度上只留下 **6 个 box**：4 面墙 + 2 个障碍。地板（顶面在 z=0）和 goal / start 贴纸（厚 1.2 cm）自动被排除。

非 box 几何会直接报错，不会被静默忽略。

| 方法 | 用途 |
| --- | --- |
| `raycast(pos, phis, range_max)` | 合成 lidar，slab 法求交，没打到返回 `+inf` |
| `distance(xy)` | 到最近 box 的 signed distance，用于判定真实碰撞 |
| `bounds(margin)` | 包住所有障碍的矩形，换 world 时用它自动定地图范围 |

## 6.3 闭环 `run_episode`

代码：[`sim/loop.py`](../nav_core/nav_core/sim/loop.py)

```python
ep = run_episode(profile, world, x0, goal, nominal,
                 safety=None,          # safety hook：safety(x, w_nom, grid) -> turn rate(s)
                 t_max=60.0, goal_radius=0.1, n_sub=10,
                 noise_std=0.0,        # lidar 距离噪声 (m)
                 grid=None,            # 自定义地图（换 world 时用）
                 tracker=None,         # 传入 ObstacleTracker 就收集事件
                 snapshot_every=None)  # 每隔多久存一份地图快照（画图用）
```

每个周期：扫描 → 建图 →（可选）追踪 → nominal → safety hook → plant 走 `n_sub` 个子步，每子步检查碰撞和到达。

返回的 `Episode`：

| 字段 | 内容 |
| --- | --- |
| `status` | `reached` / `collision` / `timeout` / `init_failed` / `filter_error` / `unsupported` |
| `t`, `traj`, `cmds` | 用时、轨迹、每周期的 `(w_nom, w)` |
| `grid`, `snapshots` | 最终地图、各时刻地图快照 |
| `events` | ObstacleTracker 的检测事件 |
| `error` | filter 停下来时的原因 |

**现在接在 safety hook 上的是 `nav_core.safety.BackupFilter`**，但 hook 只要求是个可调用对象，所以换别的 filter 很容易。

## 6.4 命令行 `run_sim.py`

```bash
cd nav_core

# 默认场景，只用 pursuit（会撞）
PYTHONPATH=. python scripts/run_sim.py

# MPPI + backup filter + 打印检测事件（需要 profile 的 V₀ 文件，见 05）
PYTHONPATH=. python scripts/run_sim.py \
    --nominal mppi --filter backup --events

# 换遮挡场景（地图范围自动按 world 定）
PYTHONPATH=. python scripts/run_sim.py \
    --nominal mppi --filter backup \
    --world ../turtlebot3/turtlebot3_controls/worlds/occluded_4x4.world \
    --x0 -0.9 -1.4 0 --goal 0.9 -0.3 --t-max 60
```

主要参数：`--robot --world --x0 --goal --t-max --noise --sensing-range --nominal --filter --unknown --v0 --events --out`

输出：终端打印结果、障碍面长度、检测事件；`outputs/` 下出一张四格图，含地图、真实障碍、障碍边、filter region 和轨迹。

## 6.5 测试场景

| World | 内容 | 用途 |
| --- | --- | --- |
| `hj_reach_target.world` | 2×2 m，4 面墙 + 2 个贴墙障碍 | 默认场景；含一条 0.4 m 窄缝 |
| `worlds/occluded_4x4.world` | 4×4 m，3 面挡墙（每面一端后藏一个箱子）+ 3 根独立柱子 | 测遮挡和孤立障碍 |
| 代码里构造的柱子场景 | 4 面墙 + 中间一根柱子 | 测 region 的 inner ring |

遮挡场景是专门为对比两种 `unknown` 模式造的：箱子藏在挡墙末端后面，机器人绕过墙端才能看到，正好制造"obstacle 进入 BRT 之前有没有被看到"的场面。

## 6.6 ROS 2 节点 `fastbrt_nav`

代码：[`ros2/fastbrt_nav/`](../ros2/fastbrt_nav/)

**和 `run_episode` 跑的是同一套逻辑**，节点本身很薄：

```
/scan, /odom → 累积地图 → nominal (pursuit | mppi) → safety filter (backup | hj | hj_backup) → /cmd_vel
```

- 每个 control period 把上个周期以来收到的每一帧 scan 积分一次，每帧都在**它自己时间戳那一刻的 LiDAR 位姿**上（`/odom` 历史插值 + profile 的 `sensor.offset`：Burger 的 LiDAR 在轮轴后方 3.2 cm），再用最新位姿算出这一周期的 turn rate；
- 用一个 sub-period timer 把 filter 返回的 4 个 zero-order-hold turn rate 依次发出去：按"距离本周期开始过了几个子步"取，不靠出队，所以两个 timer 在周期边界谁先触发都不影响；控制回调晚了就保持最后一个；
- 线速度恒为 profile 的 `v`；第一帧之前、到达 goal、filter 失败时发 0。

只用到 `LaserScan` / `Odometry` / `Twist`，**Humble 和 Jazzy 完全一样**。

### 坐标系

Gazebo 的 diff-drive odometry **以出生点为原点**。所以 launch 把 `x_pose / y_pose / yaw_pose` 同时传给机器人和节点，节点用 `odom_to_world` 换算到世界坐标（地图、goal、profile 都用世界坐标）。地图和 marker 发布在 `odom` frame。

转换函数都在 [`conversions.py`](../ros2/fastbrt_nav/fastbrt_nav/conversions.py)，**不 import rclpy**，所以在 laptop 上就有单元测试。

### 话题

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `cmd_vel` | `geometry_msgs/Twist` | 控制输出 |
| `map` | `nav_msgs/OccupancyGrid` | 累积地图（−1 未知 / 0 自由 / 100 障碍） |
| `fastbrt_markers` | `MarkerArray` | 障碍 face（蓝）、filter region（紫）、凸角（红）、凹角（橙）、拐角修正窗口（绿框） |
| `obstacle_events` | `std_msgs/String` | 每条检测事件一个 JSON：稳定 ID、kind、face 和长度、corner |

`obstacle_events` 就是给师妹 leaking-corner 模块的接口。

### 编译运行

```bash
# nav_core 和 fast_brt 不是 ROS package，走 PYTHONPATH（fast_brt 只用到 numpy 的 fast_brt.query）
export PYTHONPATH=<repo>/nav_core:<repo>/fast-brt:$PYTHONPATH

cd turtlebot3
colcon build --base-paths . ../ros2 --packages-select turtlebot3_controls fastbrt_nav
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch fastbrt_nav fastbrt_nav.launch.py
```

launch 参数：`robot goal nominal filter unknown primitives v0 corners corner_mode margin map_origin map_size log_path`，以及透传给 Gazebo 的 `world x_pose y_pose yaw_pose`。`corners`：`auto`（`hj` 开、其他关）/ `true` / `false`；`corner_mode` 默认 `thread`（拐角窗口在后台线程算，不卡控制回调）。日志 CSV 每个周期一行：t、位姿、w_nom、w、decide_s、intervened、interventions、corner_windows、corner_solves、corner_seconds。

不用 ROS 的端到端测试：`ros2/fastbrt_nav/test/test_node_without_ros.py` 用桩对象代替 rclpy，在仿真世界里把节点完整跑一遍（四种 filter）。上机步骤和测试表见 [14](14_ros2_desktop_test.md)。

### 上机前要确认的事

节点**还没有在 Gazebo 里跑过**（上机指南见 [14](14_ros2_desktop_test.md)）。第一次上机时请确认：

1. `/scan` 和 `/odom` 有数据：`ros2 topic hz /scan`；
2. 发 `/cmd_vel` 机器人会动（`Twist`；Classic 由 `gazebo_ros_diff_drive` 直接订阅，新版 Gazebo 要 bridge 里也是 `Twist` 而不是 `TwistStamped`）；
3. 决策时间小于 `ctrl_dt`（看 `log_path` 导出的 CSV 最后一列）；
4. ROS distro 和 Gazebo 版本：`echo $ROS_DISTRO`、`gazebo --version`（Classic）或 `gz sim --version`（新版）。

第 4 条：repo 里 pin 的 `turtlebot3_simulations` 是 `main@9be186f`（Jazzy、新版 Gazebo），desktop 是 **Ubuntu 22.04 + Humble**（Gazebo Classic 11）。现在两边都有 launch：`turtlebot3_controls/launch/hj_launch_world.launch.py`（新版 Gazebo + `ros_gz_bridge`）和 `hj_launch_world_classic.launch.py`（Classic + apt 的 `ros-humble-turtlebot3-gazebo`，world 开头的 `gz-sim-*` 插件在启动时去掉）；`fastbrt_nav.launch.py` 的 `gazebo` 参数选哪个，Humble 上默认 `classic`。两边的 Burger 模型 LiDAR 一样（轮轴后 3.2 cm，360 束，5 Hz）。代码已在 Humble 的 Python 组合（3.10、numpy 1.21.5、scipy 1.8.0）下跑过全部测试。

## 6.7 Legacy：旧的开环流程

`turtlebot3_controls` 里原有的 `reach_avoid.py` + `controls_keyboard.py` 保留作 baseline：用**已知完整地图**离线解一次 reach-avoid HJ，把 turn rate 存进文本文件，再**开环回放**（不读 `/odom`）。

它有几处已知的不一致（求解步长 0.05 s 但回放 0.1 s、回放发 1.0 m/s 而求解假设 0.5 m/s、墙不算障碍），README 里有记录。新的流程完全不走这条路。
