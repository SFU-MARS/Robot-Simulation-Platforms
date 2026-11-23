# 01 整体架构

## 1.1 要解决的问题

机器人在一个**事先不知道的地图**里从起点走到 goal：

- 只有一个 2D lidar，看不到的地方就是不知道；
- 机器人是 **constant-speed Dubins car**（恒速，只能转向，**停不下来**），所以"安全"不能定义成"来得及刹车"；
- 要求：**全程不碰障碍**，并且能对碰到的障碍快速估计出它的长度和拐角。

最后一点是给师妹的 leaking-corner 工作准备输入；拐角正是 primitive 组合会 leak 的地方。

## 1.2 两个 repo 的分工

|      | [FaST-BRT](https://github.com/Hu-Hanyang/FaST-BRT)（submodule，待加）         | 本 repo                                                                     |
| ---- | ---------------------------------------------------------------------------- | --------------------------------------------------------------------------- |
| 负责 | dynamics、HJ value function、primitive、safety filter、leaking-corner update | navigation：robot profile、mapping、nominal controller、ROS 2 / Gazebo 接入 |
| 依赖 | numpy / scipy，`optimized_dp` 只在离线用                                   | FaST-BRT、ROS 2                                                             |

**依赖是单向的**：本 repo import FaST-BRT，反过来不行。所以本 repo 里不出现任何 HJ 算法代码。

在线运行时**不求解任何 BRT**，value function 全部离线算好、启动时加载。在线可以 import `odp` 的 Grid / index 工具（不需要 HeteroCL），通过 `PYTHONPATH` 找到。

## 1.3 一个 control period 的数据流

主循环在 [`nav_core/sim/loop.py`](../nav_core/nav_core/sim/loop.py) 的 `run_episode`，ROS 版本在 [`nav_node.py`](../ros2/fastbrt_nav/fastbrt_nav/nav_node.py) 的 `on_control`，两者顺序完全一致：

```
          ┌─ lidar scan（仿真 raycast / 真实 /scan）
          ▼
   ① OccupancyGrid.integrate_scan      累积地图：hit / pass 计数
          │
          ├──► ② free_region            机器人可用区域（按 footprint 收缩）─┐
          │                                                                │
          ├──► ③ extract_obstacles      障碍 → face（长度）、corner       │
          │         │                                                      │
          │         └──► ④ ObstacleTracker  稳定 ID + 变化事件            │
          │                                                                │
          └──► ⑤ nominal.control        MPPI / pursuit → w_nom            │
                        │                                                  │
                        ▼                                                  ▼
                  ⑥ FastBrtGatekeeper ◄─────── primitive（来自 face）+ region
                        │   放行 w_nom，或改用上一周期存下的 witness
                        ▼
                  ⑦ /cmd_vel（4 个 sub-step 的 turn rate）
```

## 1.4 三条设计主线

### (A) 安全与感知分两层

这是 repo 里最重要的一个决定（README 的 Design decisions 1）：

| 层                    | 产物                                     | 负责                                                                |
| --------------------- | ---------------------------------------- | ------------------------------------------------------------------- |
| **region 层**   | outer ring + 每个孤立障碍的 inner ring   | **安全**：覆盖所有 detected 格子、frontier、witness 的 margin |
| **obstacle 层** | 每个障碍的 face（长度）、corner、稳定 ID | **primitive**：HJ 候选控制；以及给 leaking-corner 的拐角列表  |

关键性质：**safety 不依赖 face 拟合得好不好**。witness 的碰撞检查用的是 region 的 signed distance，而 region 是"detected 格子以外全算障碍"。face 拟合差只会让 filter 多介入几次（效率变差），不会让它误判安全。

为什么不只用 region：没有障碍身份、没有边长、没有拐角列表，而且边是在任意位置切开的，对不上"检测到障碍 → primitive → BRT"这条论文叙事。

为什么不只用 obstacle：覆盖性、遮挡、unknown 空间都要重新设计和重新证明。

### (B) 停不下来 ⇒ 安全 = 永远有后路

机器人恒速前进，不能停。所以 safety filter 的规则不是"别撞"，而是：

> 接受一个命令，当且仅当执行完之后，**仍然存在一条已验证的后路**，能让机器人无限期地安全走下去。

最简单的"无限期安全走下去"就是**绕最小圆**（loiter circle）。所以每一步都要确认：执行完这 0.2 秒后，还能开到一块足够大的已知自由空间里绕圈。

这条规则带来一个结构性限制：绕圈需要约 0.49 m 宽的空地（圆直径 0.2 m + 车身 2×0.124 m + 余量），所以**机器人永远不会进入比这更窄的通道**，即使它物理上过得去。能刹车的 Dubins4D（QBot）没有这个限制。

### (C) 两种 unknown 模式并存，用来对比

| 模式                           | unknown 空间                        | 安全性                                                   |
| ------------------------------ | ----------------------------------- | -------------------------------------------------------- |
| `unknown="occupied"`（默认） | 当作障碍，frontier 边也放 primitive | 对遮挡安全；但要求 lidar 量程 ≳ footprint + 2×转弯半径 |
| `unknown="free"`             | 当作可走，只有 detected 障碍算数    | 更激进；只有在"障碍进入 BRT 前一定被看到"时才安全        |

实验结论见 [08 实验结果](08_results.md)：量程 0.3 m 时 `free` 会撞，`occupied` 则根本启动不了（也是安全的）。

## 1.5 目录结构

```
Robot-Simulation-Platforms/
  robots/                       # 每个机器人一个 profile
    burger.yaml                 #   TurtleBot3 Burger，dubins3d，ready
    qbot.yaml                   #   Quanser QBot，dubins4d，placeholder
    crazyflie.yaml              #   Crazyflie，6D，placeholder
  nav_core/                     # 不依赖 ROS 的导航核心（可 pip 安装）
    nav_core/
      robots/                   #   profile 加载、物理 ↔ 模型单位换算
      mapping/                  #   occupancy grid、region、obstacle layer、tracking
      nominal/                  #   pursuit、MPPI
      safety/                   #   FaST-BRT filter 的 adapter（唯一 import fast_brt 的地方）
      sim/                      #   SDF world、合成 lidar、unicycle plant、闭环
    scripts/run_sim.py          # 命令行入口，出图
    tests/                      # 43 个测试
  ros2/fastbrt_nav/             # ROS 2 节点（很薄）
  turtlebot3/
    turtlebot3_controls/        #   world、launch、bridge 参数、legacy 脚本
    turtlebot3_simulations/     #   submodule（ROBOTIS），不改
  mujoco/, unitree_mujoco/      # submodule（Unitree，MuJoCo）
  docs/                         # 本文档
```

两处值得注意：

- **`nav_core` 刻意不 import `rclpy`。** 三个机器人将来跑在不同的仿真器上（Gazebo、Crazyflie 自己的 sim），核心逻辑只写一份；而且在没有 ROS 的 laptop 上就能跑完整测试。
- **`nav_core/nav_core` 是两层同名目录**，这是 Python 包的标准布局：外层是项目（放 `pyproject.toml` 和 `tests/`），内层才是被 import 的 package。`turtlebot3_controls` 也是这个模式。

## 1.6 支持的机器人

| 机器人            | Dynamics                      | 仿真器 | 状态                      |
| ----------------- | ----------------------------- | ------ | ------------------------- |
| TurtleBot3 Burger | Dubins 3D`(x, y, θ)`，恒速 | Gazebo | profile ready，全流程跑通 |
| Quanser QBot      | Dubins 4D`(x, y, θ, v)`    | Gazebo | placeholder，等 dynamics  |
| Crazyflie         | 6D                            | 待定   | placeholder，等 dynamics  |

Dynamics 由师妹在 FaST-BRT 侧提供。本 repo 只负责把 profile 里的参数接上去。

Burger 只用于**慢速的简单 case**，真正的目标平台是 QBot 和 Crazyflie。

## 1.7 当前进度

已完成：

- 不依赖 ROS 的闭环仿真，场景直接读 Gazebo 的 SDF world；
- 累积建图 + 障碍层（face、长度、拐角）+ 稳定 ID 和检测事件；
- pursuit 和 map-aware MPPI 两个 nominal controller；
- FaST-BRT gatekeeper 接入，两种 unknown 模式，两种 primitive 来源；
- ROS 2 节点 `fastbrt_nav`（**尚未在 Gazebo 里跑过**）。

待办（README 的 TODO 有完整列表）：

- 在仿真机上实跑 ROS 节点；
- 把 `RegionScene` 搬回 FaST-BRT，并把 FaST-BRT 加成 submodule（等师妹 push）；
- 和师妹确定拐角列表 / 事件的接口格式；
- QBot（4D）和 Crazyflie（6D）的 dynamics 与 profile。
