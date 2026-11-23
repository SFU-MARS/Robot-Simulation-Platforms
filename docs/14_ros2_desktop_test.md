# 14 ROS 2 上机测试指南（desktop + Gazebo）

> 日期：2026-11-23。给第一次在 desktop 上跑 `fastbrt_nav` 的同事。
> 这是这个节点**第一次在 Gazebo 里实跑**：主要目的是确认接口、时序和行为和仿真一致，发现问题就记下来，不需要调参。
> 项目背景见 [01](01_overview.md) 和导读页面；safety filter 见 [05](05_safety_filter.md)；拐角修正见 [13](13_online_local_update.md)。

## 14.1 要交回什么

1. 14.3 第 0 步环境检查的输出；
2. 14.6 测试表填好（每一行一个 run）；
3. 每个 run 的 CSV 日志（`log_path`）、终端输出、RViz 截图一张；有问题的 run 再加一份 `ros2 bag`；
4. 遇到的任何报错原样贴回来。

## 14.2 系统在做什么（一段话）

TurtleBot3 Burger 在一张**事先不知道**的地图里，从起点开到目标点。节点 `fastbrt_nav` 每 0.2 s（一个控制周期）做一次：用最新的 `/scan` 和 `/odom` 更新累积地图 → nominal controller（MPPI 或 pure pursuit）给出转向 → safety filter 决定这一周期实际的 4 个转向角速度（每 0.05 s 一个）→ 发到 `/cmd_vel`。线速度恒为 0.1 m/s，机器人**停不下来**，只能转向（转弯半径 0.1 m）。

safety filter 有三种，`filter` 参数选：

| filter | 做法 | 安全 |
| --- | --- | --- |
| `backup` | 只放行"走完之后还有一条后路"（能在已知空地里绕圈）的命令 | 有保证 |
| `hj` | 经典 HJ filter：离障碍的 BRT 太近时按 HJ 的最优控制转开；默认修正障碍拐角处的"漏"（后台线程计算） | 没有保证 |
| `hj_backup` | HJ 先给出命令，backup 检查有没有后路 | 有保证 |

## 14.3 准备

### 第 0 步：环境检查（贴回结果）

```bash
lsb_release -ds; echo $ROS_DISTRO     # 期望 Ubuntu 22.04 / humble
gazebo --version                      # 期望 Gazebo Classic 11.x
python3 --version                     # 期望 3.10
python3 -c "import numpy, scipy, contourpy, yaml; print(numpy.__version__, scipy.__version__, contourpy.__version__)"
nproc
```

**ROS 版本**：desktop 是 **Ubuntu 22.04 + ROS 2 Humble**，用 **Gazebo Classic 11**（Humble 的默认，TurtleBot3 官方的 Humble 支持也是 Classic）。`fastbrt_nav.launch.py` 在 Humble 上自动用 `turtlebot3_controls/launch/hj_launch_world_classic.launch.py`（`gazebo:=classic`）；Jazzy 上用新版 Gazebo（`gazebo:=gz`）。Classic 版 launch 是这次新写的，**还没在真机上跑过**，有问题请原样贴回来。

安装（Humble 用系统 Python 3.10；numpy 1.21.5、scipy 1.8.0 是 jammy 自带的版本，代码已在这个组合下跑过全部测试）：

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description \
                 python3-numpy python3-scipy python3-yaml python3-pip
pip3 install --user "contourpy<1.1"   # jammy 没有 python3-contourpy；1.0.x 支持 numpy 1.21
```

### 第 1 步：代码和 V₀

```bash
git clone --recurse-submodules -b add_lidar git@github.com:SFU-MARS/Robot-Simulation-Platforms.git
cd Robot-Simulation-Platforms
git log --oneline -1                      # 记下 commit
cd nav_core && python3 scripts/fetch_v0.py && cd ..   # 下载离线 value function V₀（14 MB，GitHub release）
```

`fetch_v0.py` 会校验 sha256，最后一行应该是 `fetched ...` 或 `ok ...`。

### 第 2 步：不用 ROS 的自检（约 2 分钟）

```bash
cd nav_core
PYTHONPATH=. python3 -m pytest -q tests                              # 期望 58 passed
PYTHONPATH=. python3 scripts/run_sim.py --nominal mppi --filter backup   # 期望 reached at t ≈ 17.7 s
cd ../ros2/fastbrt_nav
PYTHONPATH=.:../../nav_core python3 -m pytest -q test               # 期望 13 passed
cd ../..
```

最后一个测试用桩对象代替 rclpy，在仿真世界里像 Gazebo 一样带时间戳地发 `/odom`（每 0.05 s）和 `/scan`（每 0.2 s，晚于控制周期），把节点完整跑一遍（四种 filter 都到达目标，每帧扫描都在真实的 LiDAR 位姿上积分）。通过了说明 Python 这边没问题，后面的问题就在 ROS / Gazebo 这一层。

### 第 3 步：编译

```bash
export PYTHONPATH=$PWD/nav_core:$PWD/fast-brt:$PYTHONPATH   # nav_core 和 fast_brt 不是 ROS 包；每个新终端都要设
cd turtlebot3
colcon build --base-paths . ../ros2 --packages-select turtlebot3_controls fastbrt_nav
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

只编译 `turtlebot3_controls` 和 `fastbrt_nav`：仓库里的 `turtlebot3_simulations` 子模块是 Jazzy 版的，Humble 上用 apt 装的 `ros-humble-turtlebot3-gazebo`，不要编译子模块。

先单独确认 Gazebo 能起来（应该看到 2 × 2 m 的场地和一台 Burger，`ros2 topic hz /scan` 约 5 Hz）：

```bash
ros2 launch turtlebot3_controls hj_launch_world_classic.launch.py x_pose:=-0.8 y_pose:=-0.8 yaw_pose:=0.2
```

world 文件是给新版 Gazebo 写的，开头三行 `gz-sim-*` 插件 Classic 加载不了；Classic 版 launch 会把它们去掉、另存到 `/tmp/classic_<world>` 再加载，其他内容（墙、箱子、地面）不变。

## 14.4 运行

每个 run 开一个终端（已 `source install/setup.bash`、设好 `PYTHONPATH` 和 `TURTLEBOT3_MODEL`）：

```bash
ros2 launch fastbrt_nav fastbrt_nav.launch.py <参数> log_path:=$HOME/fbn_<编号>.csv
```

另开一个终端看话题和 RViz：

```bash
ros2 topic hz /scan                    # 期望约 5 Hz（LDS-01）
ros2 topic hz /cmd_vel                 # 期望约 20 Hz（每 0.05 s 一个转向）
ros2 topic echo /obstacle_events       # 障碍检测事件（JSON）
rviz2                                  # Fixed Frame: odom；Add → By topic: /map（Map）、/fastbrt_markers（MarkerArray）
```

RViz 里的 marker：蓝线 = 检测到的障碍面，紫线 = filter 用的自由区域，红 / 橙点 = 凸 / 凹拐角，**绿框 = 拐角修正窗口**（只有 `hj` 默认有）。

需要录包时：

```bash
ros2 bag record /scan /odom /cmd_vel /map /fastbrt_markers /obstacle_events -o $HOME/fbn_<编号>_bag
```

### launch 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `world`, `x_pose`, `y_pose`, `yaw_pose` | hj_reach_target，(−0.8, −0.8, 0.2) | Gazebo world 和出生位姿（也传给节点：odometry 从出生点开始） |
| `goal` | `[0.7, 0.2]` | 目标点（世界坐标，m） |
| `nominal` | `mppi` | `mppi` / `pursuit` |
| `filter` | `backup` | `none` / `backup` / `hj` / `hj_backup` |
| `unknown` | `occupied` | 未知区域当障碍（`occupied`）还是当空地（`free`） |
| `corners` | `auto` | 拐角修正：`auto`（`hj` 开，其他关）/ `true` / `false` |
| `corner_mode` | `thread` | 拐角窗口在后台线程计算（在线用这个） |
| `margin` | `-1` | `hj` / `hj_backup` 的距离余量（m），< 0 用默认（`hj` 0，`hj_backup` 0.02） |
| `map_origin`, `map_size` | profile 的 | 地图范围（遮挡场景要设） |
| `log_path` | 空 | CSV 日志路径 |

### LiDAR 接口（节点怎么用 `/scan`）

| 项 | Gazebo 里的 Burger（`turtlebot3_gazebo` 的 `model.sdf`） | 节点 / profile |
| --- | --- | --- |
| 话题 | Classic（Humble）：`gazebo_ros_ray_sensor` 插件直接发 `/scan`；新版 Gazebo（Jazzy）：`gpu_lidar` → `ros_gz_bridge` → `/scan`。都是 `sensor_msgs/LaserScan`，frame `base_scan` | 订阅 `scan`，QoS 用 sensor data（best effort，能收 reliable 的发布） |
| 光束 | 360 束，`min_angle` 0、`max_angle` 6.28，5 Hz | 角度按消息里的 `angle_min + i·angle_increment` 算，不假设束数 |
| 量程 | 0.12–3.5 m，噪声 σ = 1 cm，分辨率 1.5 cm | 软件截断到 profile 的 0.8 m：≥ 0.8 m 或 `+inf` 当作"这条光束 0.8 m 内都是空的"，`nan` / `-inf` / < 0.12 m 丢掉（REP 117） |
| 安装位置 | `lidar_joint` 在轮轴中心（`base_footprint`）后方 **3.2 cm**、高 17.1 cm | profile `sensor.offset: [-0.032, 0.0]`：每帧扫描从 LiDAR 的实际位置积分（仿真也一样） |
| 时间 | 扫描带时间戳（仿真时间，`use_sim_time` 默认 true）；`/odom` Classic 约 30 Hz、新版 Gazebo 约 50 Hz | 每帧扫描按它的时间戳在 `/odom` 历史里插值出当时的位姿；控制周期里把上个周期以来收到的每一帧都积分一次（不重复、不漏）。日志的 `scan_age` 是扫描比控制晚了多久 |

为什么要按时间戳对齐：扫描 5 Hz、控制 5 Hz，两者相位随机，扫描被用到时可能已经过了约 0.2 s；机器人以 1 rad/s 转向时，用"最新位姿"积分会让 0.8 m 远的点偏 **约 11 cm**（不用 ROS 的端到端测试里实测 108.6 mm），对齐后为 0。

**机器人半径**：Burger 底盘是 14 cm 的方盒，中心在轴心后方 3.2 cm，后角离轴心 √(0.102² + 0.07²) = **12.4 cm**（轮子外沿 9.5 cm）。位姿在轴心，所以 profile 的 `footprint_radius` 取 **0.124 m**：以轴心为圆心、包住整台车的圆，转向时也不会有部分扫出去。下面的"最近距离"都是从这个圆量起。

## 14.5 测试项和预期

预期来自不用 ROS 的端到端测试（`test/test_node_without_ros.py` 里模拟 Gazebo 的那套：同一个节点，每 0.05 s 一次里程计、5 Hz 带时间戳的扫描，机器人半径 0.124 m，2026-11-23）。Gazebo 的里程计、扫描时序和物理不完全一样，**结果有差别是正常的**，但"撞 / 不撞""到 / 没到"应该一致。

| # | 场景 | 参数 | 仿真里的预期 |
| --- | --- | --- | --- |
| T1 | hj_reach_target | `nominal:=pursuit filter:=none` | **约 7.0 s 撞上** obstacle 2 的角（证明不加 filter 会撞；撞上后可以 Ctrl-C） |
| T2 | hj_reach_target | `nominal:=pursuit filter:=backup` | 到达，约 18.2 s，约 12 次介入，最近 4.1 cm |
| T3 | hj_reach_target | `filter:=backup`（MPPI） | 到达，约 18.2 s，0 次介入，最近 7.7 cm |
| T4 | hj_reach_target | `nominal:=pursuit filter:=hj` | 到达，约 18.0 s，约 11 次介入，最近 1.1 cm（贴得很近） |
| T5 | hj_reach_target | `filter:=hj_backup`（MPPI） | 到达，约 19.6 s，约 19 次介入，最近 5.1 cm |
| T6 | occluded_4x4 | `filter:=backup` + 下面的遮挡参数 | 到达，约 22.8 s，0 次介入，最近 12.5 cm |
| T7 | occluded_4x4 | `filter:=hj` + 下面的遮挡参数 | 到达，约 22.8 s，0 次介入，最近 12.5 cm |

遮挡场景的参数：

```bash
world:=$(ros2 pkg prefix turtlebot3_controls)/share/turtlebot3_controls/worlds/occluded_4x4.world \
x_pose:=-0.9 y_pose:=-1.4 yaw_pose:=0.0 goal:="[0.9, -0.3]" map_origin:="[-2.1, -2.1]" map_size:="[4.2, 4.2]"
```

## 14.6 测试表（填好交回）

| # | 结果（reached / collision / stopped / 卡住） | 用时 (s) | 介入次数（CSV 最后一行 `interventions`） | 最大 `decide_s` | 周期间隔的最大值（相邻 `t` 之差） | 备注 |
| --- | --- | --- | --- | --- | --- | --- |
| T1 | | | | | | |
| T2 | | | | | | |
| T3 | | | | | | |
| T4 | | | | | | |
| T5 | | | | | | |
| T6 | | | | | | |
| T7 | | | | | | |

从 CSV 算两列的一行命令：

```bash
python3 -c "import csv,sys; r=list(csv.DictReader(open(sys.argv[1]))); t=[float(x['t']) for x in r]; print('max decide_s', max(float(x['decide_s']) for x in r), 'max period', max(b-a for a,b in zip(t,t[1:])), 'interventions', r[-1]['interventions'], 'corner windows used (max)', max(int(x['corner_windows']) for x in r))" $HOME/fbn_T4.csv
```

### 每个 run 要看的

- **扫描**：`scan_age` 应在 0–0.2 s 之间（扫描比控制晚多少）；如果持续大于 0.2 s 或一直为同一个值，说明扫描没有按时到达。
- **时序**：`decide_s` 应远小于 0.2 s（仿真里 `backup` 约 20–40 ms，`hj` 约 10 ms）；相邻 `t` 的间隔应约等于 0.2 s。`hj` 的拐角窗口在后台线程算，如果间隔明显变大（> 0.3 s），说明后台线程拖慢了控制回调，请记下来。
- **`hj`**：CSV 的 `corner_windows` 在机器人靠近墙角时应大于 0，RViz 里能看到绿框。
- **行为**：机器人是否沿着仿真里的路线走；有没有原地打转很久（`backup` 在窄缝前会绕圈，这是预期的）。
- **坐标**：RViz 里蓝线（检测到的面）应贴在 Gazebo 里障碍的外侧约 12 cm（机器人半径 + 一格）；如果整体错位，多半是出生位姿参数和 Gazebo 实际的不一致。

## 14.7 常见问题

| 现象 | 原因 / 处理 |
| --- | --- |
| `no V0 file at ...` | 没下载 V₀：`cd nav_core && python3 scripts/fetch_v0.py` |
| `fast_brt.query is not importable` | `PYTHONPATH` 没包含 `<repo>/fast-brt`，或者误放了旧的 Hu-Hanyang/FaST-BRT |
| `... does not match the sha256` | V₀ 文件被替换过：重新 `fetch_v0.py` |
| 机器人不动，但 `/cmd_vel` 有数据 | Classic：`ros2 topic info /cmd_vel -v` 看 `gazebo_ros_diff_drive` 是否订阅了（类型 `geometry_msgs/msg/Twist`）；新版 Gazebo：bridge 里的类型（`Twist` / `TwistStamped`）不匹配，见 `turtlebot3_controls/params/turtlebot3_burger_bridge.yaml` |
| 节点日志 `filter stopped the robot: no witness at the start` | `backup` / `hj_backup` 在起点找不到后路（第一帧扫到的空地放不下绕圈的圆）。检查 `/scan` 是否正常、出生点是否离墙太近 |
| 节点日志 `robot is outside the free region` | 机器人离地图上的障碍太近（擦边）；记下时间和位置 |
| 地图和 Gazebo 错位 / 旋转 | 出生位姿 `x_pose y_pose yaw_pose` 和 world 不一致；odometry 是从出生点算起的 |
| launch 找不到 `gazebo_ros` / `turtlebot3_gazebo` | `sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-turtlebot3-gazebo` |
| launch 报 `ros_gz_sim` 找不到 | 用的是新版 Gazebo 的 launch：Humble 上加 `gazebo:=classic`（`ROS_DISTRO=humble` 时默认就是） |
| Gazebo 第一次启动很久没反应 | Classic 第一次会去联网下载模型库；可以先 `export GAZEBO_MODEL_DATABASE_URI=""` 再启动 |
| `gzserver` 起不来 / 端口占用 | 上一次的进程没退：`pkill -f gzserver; pkill -f gzclient` |
| `ModuleNotFoundError: contourpy` | `pip3 install --user "contourpy<1.1"` |
