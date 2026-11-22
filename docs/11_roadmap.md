# 11 路线图：已完成、进行中、待办

> 更新：2026-11-21。这一篇是整个项目的进度总表，细节在各篇文档里。

## 11.1 目标

对 Dubins3D、Dubins4D，以及未来的 6D 无人机：在**未知环境**里用 nominal controller 导航，边走边建图，用 **2D primitive** 的 BRT 保证安全；primitive 组合在拐角处 leak 时，用 **online local update** 修正。

论文主线：**2D primitive（离线一次）+ 在线 local update 修 leaking corner**。filter 有三种做法，都保留为选项。

## 11.2 已定的决策

| 决策             | 内容                                                                                                                                                                   | 出处                                       |
| ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------ |
| repo 分工        | primitive / value function 相关的放**fast-brt**（我们自己的 `robot-sim` 分支），感知、建图、filter 和 ROS 放 **Robot-Sim**；`Hu-Hanyang/FaST-BRT` 退役 | [09](09_fast_brt_integration.md)            |
| primitive 放置   | **内移**：矩形朝向自由空间的边和检测到的面重合，BRT 只由长度 L 决定，长面不切段                                                                                  | [10 §10.2](10_primitive_alpha_accuracy.md) |
| 离线`V₀`      | **线性编码 + odp `accuracy="medium"` + 241 网格 + α=20 + BRT**                                                                                                | [10 §10.3](10_primitive_alpha_accuracy.md) |
| 短面             | 短于`L_min ≈ 2·dx·√α`（约 0.45 R）的面，对称延长到 `L_min`                                                                                                    | [10 §10.1](10_primitive_alpha_accuracy.md) |
| primitive 用 BRT | BRS 在面前方的高度为 0，不能用于避障                                                                                                                                   | [10 §10.3](10_primitive_alpha_accuracy.md) |
| filter           | 三种都保留为选项：`hj`（经典 HJ filter）、`backup`（后路验证，原 witness gatekeeper）、`hj_backup`                                                               | [05](05_safety_filter.md)                   |
| 3D 动力学        | 先做恒速 Dubins3D，参数以后再扫；4D 一定要做                                                                                                                           | —                                         |
| 地图             | 计数模型（τ=0.2）+ 擦边保护                                                                                                                                           | [03](03_mapping.md)                         |
| 障碍             | 连在一起的算一个；面 = 包裹直线；稳定 ID + 事件                                                                                                                        | [03](03_mapping.md)                         |

## 11.3 已完成

- **Robot-Sim**
  - 不依赖 ROS 的闭环仿真（读 Gazebo SDF world），pursuit 和 MPPI 两个 nominal；
  - 计数地图、可用区域（含孤立障碍）、障碍层（面、长度、拐角）、追踪和检测事件；
  - backup filter 接入（目前借用 FaST-BRT 的代码），两种 unknown 模式，内移 / 对称两种放置；
  - ROS 2 节点 `fastbrt_nav`（**未在 Gazebo 实测**）；
  - α 实验和闭环对比、遮挡场景、文档 01–10。
- **fast-brt（`robot-sim` 分支，`539d0c6`）**
  - G1：不装 HeteroCL 也能 `import fast_brt`；
  - 精度复核脚本，已在 mars-14 上跑完。

## 11.4 进行中（下一步，已同意）

| # | 任务                                                                                                                                                                       | 在哪      | 依赖 |
| - | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------- | ---- |
| 1 | **导出离线 `V₀`**：新增 CLI，按 config 求解单个 primitive 的 **BRT**（可选 `accuracy`），只保存最后一层和网格参数                                         | fast-brt  | —   |
| 2 | **在线点查询**（纯 numpy，不 import odp）：长度平移、内移放置、短面延长、`V`、`∂V/∂θ`、`u*`                                                                 | fast-brt  | —   |
| 3 | **在 mars-14 上导出推荐配置的 `V₀`**                                                                                                                              | mars-14   | 1    |
| 4 | **把 backup filter 从 FaST-BRT 迁到 Robot-Sim**（`nav_core/safety`），改名 `BackupFilter`；HJ 项改用第 2 步的点查询。两个包都叫 `fast_brt`，不迁移就没法同时用 | Robot-Sim | 2    |
| 5 | **切换默认值**（内移、不切段、加载 `V₀` 文件），重跑批量实验，更新文档                                                                                            | Robot-Sim | 3、4 |

## 11.5 待办

**filter 和 local update**

- [ ] `hj` 和 `hj_backup` 两种 filter mode（`hj` 最简单）；
- [X] local update 的 BRT 版本（HJ_local_update `robot-sim` 分支，`brt=True`）：每个被更新的点再取 `min(V, V(t=0))`；numpy 版在本地验证过，和整体 BRT 求解**符号完全一致**（L 形两条线：并集漏 74 格，local update 后 0 格）；fast-brt 新增 `target_set_mode="brt"` 和 `local_update_impl="numpy"`；
- [ ] 在 mars-14 上验证 HeteroCL 版的 BRT local update，并测实时性（`pytest -m slow -k tube`、`experiments/local_update_timing.py`）；
- [ ] local update 目前只能用一阶格式（和 primitive library 的格式必须一致，否则修正区域会扩张到整张网格）；要用二阶，得改 kernel（ENO2 + RK2）；
- [ ] local update 接入在线流程：fast-brt 服务模式或 numpy 版直接在线跑（先比较 numpy 和 HeteroCL 的速度）；随机器人移动的窗口（G4）、按障碍 ID 增量更新（G5）；
- [ ] 对比实验：只用 transport vs transport + local update（介入率、超时、后路搜索开销），α 对 `hj` filter 的影响。

**动力学和机器人**

- [ ] Dubins4D（QBot）：profile、MPPI 的 4D step、4D 的后路（终端集合是"停下来"）、QBot 的 Gazebo 模型；
- [ ] 6D 无人机：确认模型（HJ_local_update 的 `quadrotor_6d` 是竖直平面四旋翼，不是水平面导航）、传感器、仿真器；
- [ ] Dubins3D 的参数扫描（是否改用 `Dubins3D2Ctrl`）。

**工程**

- [ ] 在 mars-14 / 仿真机上实跑 ROS 节点（检查清单见 [06](06_simulation_and_ros.md)）；
- [ ] 3D 点云 → 2D 扫描（`PointCloud2` 取高度带投影）；
- [ ] 启动时检查 lidar 量程是否够 `unknown="occupied"` 启动；
- [ ] 拐角列表 / 事件格式和 local update 的输入对齐；
- [ ] odp 版本 pin（fast-brt 依赖带 `dims` 参数的旧版 `Grid`）、HJ_local_update 作为 fast-brt 的依赖。

## 11.6 待讨论的问题

1. **local update 和 BRT**：local update 现在是 BRS。只把最后一层换成 BRT 不等价，见下面的说明。
2. **local update 的定位**：在线每周期跑，还是按事件触发、异步更新？numpy 版够不够快？
3. **6D 无人机用哪个模型**。

### 为什么"只替换 BRT 的最后一层"不够

BRT 要求"不论怎么控制，**某个时刻**都会撞上"；把各个时刻的 BRS 取并集，得到的是"**存在某个固定时刻**，不论怎么控制都在障碍里"。量词顺序不同：

$$
V_{\text{BRT}}(x) = \max_{u}\ \min_{t \le T} \ell(\xi(t)), \qquad
\min_{t \le T} V_{\text{BRS}}(x, t) = \min_{t \le T}\ \max_{u}\ \ell(\xi(t)) \ \ge\ V_{\text{BRT}}(x)
$$

所以用 BRS 拼出来的集合**只会比 BRT 小**（不安全的方向）。对细长的 primitive 尤其明显：车可以"穿墙而过"，任何一个固定时刻都不在矩形里，但途中撞上了。精度复核里，BRS 在面前方的高度正好是 0。

正确的做法是在 local update 的**每个时间步**里都取 `min(V, ℓ)`（odp 的 `minVWithV0`），改动不大，但需要用"local update vs 整体 BRT 求解"来验证。
