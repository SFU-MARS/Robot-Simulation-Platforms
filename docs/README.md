# Robot-Simulation-Platforms 代码文档

这个 repo 做的事情：**机器人在未知地图里用 lidar 边走边建图，同时用 FaST-BRT 的 HJ reachability safety filter 保证不撞。**

机器人事先不知道障碍在哪。每个 control period 它扫描一次、更新自己的地图、估计障碍的长度和拐角、让 nominal controller 提出一个转向命令，再由 safety filter 决定放行还是改用备用方案。

## 阅读顺序

| 文档 | 内容 |
| --- | --- |
| [01 整体架构](01_overview.md) | 两个 repo 的分工、数据流、三条设计主线、目录结构 |
| [02 Robot profile](02_robot_profile.md) | `robots/*.yaml`、单位换算、运动能力检查 |
| [03 Mapping](03_mapping.md) | occupancy grid、free region、obstacle layer、obstacle tracking |
| [04 Nominal controller](04_nominal.md) | pursuit、MPPI、cost-to-go |
| [05 Safety filter](05_safety_filter.md) | gatekeeper、witness、**2D primitive**、RegionScene |
| [06 仿真与 ROS 2](06_simulation_and_ros.md) | `nav_core.sim`、SDF world、`fastbrt_nav` node |
| [07 MPPI 完整案例](07_mppi_walkthrough.md) | **从加载 map 到 controller 到 2D primitive，走完一个周期** |
| [08 实验结果](08_results.md) | 各组 batch 实验的数字和结论 |
| [09 接入 fast-brt（讨论稿）](09_fast_brt_integration.md) | SFU-MARS/fast-brt 的现状、差距清单、复用方案、待讨论问题 |

只想快速上手：读 [01](01_overview.md) 和 [07](07_mppi_walkthrough.md) 就够。

## 术语对照

| 术语 | 含义 |
| --- | --- |
| **nominal controller** | 只管往 goal 走、不保证安全的控制器（pursuit 或 MPPI） |
| **safety filter / gatekeeper** | 检查 nominal 命令，不安全就拒绝，改用已验证的备用方案 |
| **witness** | 一条已验证的"后路"：一段 rollout + 末端的 loiter circle，全程不碰障碍 |
| **loiter circle** | 恒速机器人"原地等待"的唯一方式：方向打死绕最小圆 |
| **primitive** | FaST-BRT 的 facing-side primitive，一条障碍边对应一个，见 [05](05_safety_filter.md) |
| **region** | 机器人可用的自由区域（outer ring + 孤立障碍的 inner rings） |
| **face** | 障碍某一面的包裹直线，长度即该面的估计长度 |
| **BRT** | Backward Reachable Tube，注定要撞的状态集合 |

## 代码入口

```bash
cd nav_core

# 只用 nominal controller（会撞）
PYTHONPATH=. python scripts/run_sim.py --nominal mppi

# 加上 safety filter（需要 fast_brt）
PYTHONPATH=.:<FaST-BRT 路径> python scripts/run_sim.py --nominal mppi --filter gatekeeper --events

# 测试
PYTHONPATH=. python -m pytest -q tests
```
