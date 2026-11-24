# 15 QBot（Dubins 4D）：从离线 V₀ 到在线导航

2026-11-23 开始。Burger（恒速 Dubins 3D）的整条链路跑通以后，把同一套方法搬到 QBot：车能加速、刹车、倒车，状态多了速度 v 这一维。

## 15.1 机器人和单位

**QBot = Quanser QBot Platform**（hj_nav 论文的实验平台）。

| 项 | 值 | 来源 |
| --- | --- | --- |
| 底盘 | 六边形，对边 570 mm，高 227 mm；两轮差速 + 万向轮 | Quanser 规格表 |
| 机器人半径 `footprint_radius` | 0.285 m（以两驱动轮中点为圆心） | 同上；QLabs：底盘坐标系在两驱动轮中间 |
| 轮距 | 0.390 m | QLabs QBot Platform 文档 |
| LiDAR | 在底盘坐标系前方 0.221 m、离地 0.207 m；每秒 16,800 点 | QLabs、规格表 |
| 加速度 a | [−0.5, 0.5] m/s² | hj_nav 论文 `sections/Simulation.tex` |
| 转向率 ω | [−0.5, 0.5] rad/s | 同上 |
| 速度 v | [−0.2, 0.8] m/s | 同上 |

profile 在 `robots/qbot.yaml`：软件截断量程 2.0 m，控制周期 0.1 s（10 Hz），地图分辨率 4 cm。

**模型单位**（fast-brt 解 V₀ 用的无量纲单位）：时间单位 1/ω_max = 2 s，长度单位 v_max/ω_max = 1.6 m。于是 ω_max = 1、v_max = 1、v_min = −0.25、a_max = 1.25。速度的单位是 1.6 m / 2 s = 0.8 m/s，所以 −0.2 m/s 在模型里是 −0.25（第一版配置误写成 −0.125，已改正）。

**状态顺序**：nav_core 里 4D 的状态是 (x, y, θ, v)，前三维仍是位姿，建图、LiDAR、ROS 的代码都不用改；fast-brt 的 V₀ 是 (x, y, v, θ)，只在查询 V₀ 时换顺序（`Scene._v0_states`）。控制是 (a, ω)。

## 15.2 速度上下限怎么进 HJ

odp 的 `DubinsCar4D` 对 v 没有任何限制。有两种做法：

| 做法 | 怎么做 | 问题 |
| --- | --- | --- |
| **动力学限幅**（采用） | v 到了 v_min 就不能再减速、到了 v_max 就不能再加速（HJ_local_update 的 `dubins_4d`）；v 轴正好是 [v_min, v_max] | 疑点只有 v 边界上外推的虚拟格点，已验证没影响 |
| hj_nav 的速度带 | v 轴取宽，把 v < v_min − 0.05 和 v > v_max + 0.05 两条带子当障碍并进 avoid set，再乘 λ = 10⁴ | 单位混用（m/s 和 m² 取 min），λ = 1 时不安全集被数值误差撑大约 1 m（`~/Codes/hj_nav/BRT_payoff_fix_record.md`）；还允许用到 0.85 m/s |

`fast-brt/experiments/speed_limits_4d.py` 在半平面墙和一个 primitive 线段上比了三种设置（限幅、限幅且 v 轴加宽、λ 速度带），以 rollout（枚举一大批 bang-bang 控制序列模拟出的最好结果）为真值：**三者的值相差 < 10⁻⁴**。限幅最简单，V 一直是长度单位，所以用它。

同一实验发现一阶格式（odp 的 `low`）在粗网格上对少数状态**偏乐观**，集中在 v = v_max、加速冲过线段端点的那类逃法上：

| 网格（模型单位） | 误判为安全的比例 | v_max 处偏差的中位数 |
| --- | --- | --- |
| dx 0.05，θ 36，v 11 | 0.10% | +0.0073 |
| dx 0.05，θ 72，v 21 | 0.17% | +0.0075 |
| dx 0.025，θ 36，v 11 | 0.03% | +0.0017 |

误差主要随 x、y 的网格收敛，θ、v 加密几乎没用。α 越大误差越大（α 5 / 10 / 20 / 40：误判 0.03 / 0.00 / 0.10 / 0.23%），差别不大，**4D 也用 α = 20**，和 3D 一致。

## 15.3 离线 V₀

`fast-brt/configs/v0_dubins_4d.json`：

- **时域 T = 1.6**（3.2 s）：numpy 实测，t = 1.6 时值的变化 < 10⁻⁵，1.81 以后不再变化（3D 是 2）。能刹车，所以收敛得快。
- **网格** 385 × 385 × 11 × 36：x、y ∈ [−4.8, 4.8]，取 3T 宽，使长面截断长度 l_ref ≥ 2vT（长面中间段的值才精确）；dx = 0.025（4 cm）；短于 l_min = 0.224（36 cm）的面延长到 l_min。
- mars-14 上解 143 s、192 步，文件 235 MB，在 release `v0-2026-11-23`（`robots/v0_release.yaml`，`python scripts/fetch_v0.py --all`）。

**两项核对**：

1. **odp 和 numpy 一致**：97 × 97 的小配置（`v0_dubins_4d_coarse.json`），odp 解的和 `export-v0 --solver numpy`（`fast_brt.corner.Stepper4D`，逐位等于 HJ_local_update 的 `value_change_numpy`）解的，最大差 1.9 × 10⁻⁶（float32 舍入），零水平集附近没有一个点符号不同。
2. **物理上合理**：正对一面墙全速冲过去，能躲开的最近距离：

| 速度 | V₀ | 只直线刹车需要 |
| --- | --- | --- |
| 0.8 m/s | 0.636 m | 0.640 m |
| 0.6 m/s | 0.388 m | 0.360 m |
| 0.4 m/s | 0.192 m | 0.160 m |

全速时边刹边转能多躲一点；低速时多出的 2–3 cm 是一阶格式往安全一侧的误差。最优控制 u* 是全力刹车加转向。

## 15.4 在线：nav_core 改了什么

| 部分 | 3D（Burger） | 4D（QBot） |
| --- | --- | --- |
| 车（`robots/dynamics.py`） | 恒速圆弧 | 精确解：v 线性变化到上下限为止，位置是 v(cos θ, sin θ) 的解析积分 |
| pursuit | 转向对准目标 | 同上，期望速度 v_max·cos(航向误差)，近目标减速，加速度去追 |
| MPPI | 采样 ω | 采样 (a, ω)；软障碍代价的衰减距离放宽到 filter 的 clearance + 2 cm（见下） |
| 后路（witness） | 进入一个空闲的 loiter circle 绕圈 | **刹停**：每个子步 a = clip(−v/dt, ±a_max)，速度恰好停在 0；候选之间只是转向不同（HJ 的闭环转向，再试 ±1、0、±0.5）。停下就永远安全（障碍不动、地图只会多出空地） |
| `hj` / `hj_backup` | HJ 逃逸 ω = sign(∂V/∂θ) | (a, ω) = (a_max sign(∂V/∂v), sign(∂V/∂θ)) |
| 拐角修正 | 有（[13](13_online_local_update.md)） | **还没有**：`hj` 默认关，要开会报错 |

**后路的建模障碍只用可行区域**（`rect_margin`，4D 默认 False）。backup filter 检查后路时，原来的建模障碍是"可行区域之外 ∪ primitive 矩形"。可行区域是地图做距离变换得到的（离任何非空闲格都超过机器人半径的点），本身就是圆角的真实 C 空间，单独用已经安全；矩形只是更保守。对半径 0.285 m 的 QBot，矩形太保守了：

- face 两端各延长一个车半径（方角）；
- 一段边界挨着未知区时，它断开的一端也延长一个车半径；
- 矩形的厚度在面端点之外伸进了空地。

实测：车在离真实障碍 9.7 cm 的地方停下，地图更新后 L 形底边的矩形长到了车身上（建模余量 −0.02 m），从此再也找不到后路，永远停着。只用可行区域以后，MPPI + backup 在竞技场里基本不再介入。3D 的默认值不变。

**MPPI 和 filter 要对得上**：MPPI 原来只对离障碍 3 cm 以内重罚，会规划一条贴边的路，而 filter 要求留 6 cm（1.5 个栅格）的 clearance；4D 的后路是停下，停下后 MPPI 还提同一条路，就僵住了。所以 QBot 的 MPPI 软惩罚衰减距离改成 clearance + 2 cm。

## 15.5 地图

`nav_core/scripts/make_qbot_worlds.py` 生成，放在 `turtlebot3/turtlebot3_controls/worlds/`：

- **`qbot_arena.world`**：照 Quanser 的 3.6 m × 3.6 m 场地。中间一个 L 形（凹角朝左上），西北角附近一根柱子。
- **`qbot_occluded.world`**：6 m × 6 m。三面挡墙，每面一端背后藏一个箱子；两根柱子。

QBot 宽 0.57 m，障碍又按方角膨胀，所以通道要么约 1 m 宽，要么彻底堵死（小于 0.75 m 的只出现在挡墙背后的口袋和柱子与墙角之间）。最早一版用了 0.75–0.8 m 的通道，扣掉车宽和两侧 clearance 只剩约 6 cm 的活动余量，filter 几乎一直在拦。

## 15.6 结果

批量（`filter_batch.py --robot qbot --filters none backup hj hj_backup --nominals pursuit mppi --t-max 60`，unknown occupied，到达半径 0.2 m，2026-11-23），到达 / 碰撞 / 超时，括号里是介入率；原始数据 [`figures/batch_qbot.csv`](figures/batch_qbot.csv)：

| 场景 | nominal | 不加 filter | `backup` | `hj` | `hj_backup` |
| --- | --- | --- | --- | --- | --- |
| 竞技场（40 次） | pursuit | 14 / **26** / 0 | 20 / 0 / 20（0.52） | 28 / **2** / 10（0.24） | 14 / 0 / 26（0.60） |
| | MPPI | 34 / 0 / 6 | 34 / 0 / 6（0.0） | 34 / 0 / 6（0.019） | 29 / 0 / 11（0.099） |
| 遮挡 6 × 6（35 次） | pursuit | 8 / **27** / 0 | 11 / 0 / 24（0.67） | 14 / **2** / 19（0.36） | 9 / 0 / 26（0.66） |
| | MPPI | 35 / 0 / 0 | 35 / 0 / 0（0.0） | 32 / 0 / 3（0.043） | 24 / 0 / 11（0.15） |

- **`backup` 和 `hj_backup` 都没有碰撞**；`hj` 跟着 pursuit 会撞（凹角处漏，4D 还没有拐角修正）。
- **MPPI 本身就不撞**，filter 几乎不介入（`backup` 介入率 0），到达数和不加 filter 一样；竞技场剩下的 6 次超时不加 filter 也有，是 MPPI 停在目标附近（障碍软惩罚和到目标的代价平衡住了），不是 filter 造成的。
- **`hj_backup` 超时更多**（MPPI：竞技场 11、遮挡 11）：HJ 在离 BRT 2 cm 时就开始逃逸（刹车加转向），这和 3D 时一样（[05](05_safety_filter.md) §5.8）。
- **pursuit 加 filter 超时很多**：见 15.7 第一条。
- 计算：每步 10–25 ms（不含拐角修正），控制周期 100 ms。

第一轮批量用的是 Burger 的到达半径 0.1 m，终点之一离北墙只有 0.4 m：MPPI 有 17 次停在离目标 0.13–0.31 m 处被算成超时。改成 0.2 m、终点挪到 (−0.4, 1.2) 后重跑，就是上表。

单次（`run_sim.py --robot qbot`，竞技场，MPPI）：`backup`、`hj`、`hj_backup` 都到达、不撞；没有 filter 时 pursuit 冲进 L 形。`hj` 加 pursuit 从 L 形凹角里出发会撞：和 3D 一样，经典 `hj` 在凹角处漏，4D 还没有拐角修正。

## 15.7 已知问题和下一步

- **pursuit 容易僵住**：pursuit 不看地图，一直朝障碍冲；4D 的后路是停下，停下以后 pursuit 还是同一个方向，于是一直停着（超时）。3D 不会这样，因为后路是绕圈，车一直在动。这是名义控制器的问题。
- **4D 拐角修正**：`Stepper4D` 已经写好（逐位等于 `value_change_numpy`），窗口的稳态 local update 还没接，计算量比 3D 大（多一维 v）。
- **ROS 节点**：`fastbrt_nav` 还只支持 3D。4D 要从 `/odom` 读速度，把 (a, ω) 积分成 `cmd_vel` 的 (v, ω)，还需要一个 QBot 的 Gazebo 模型。
