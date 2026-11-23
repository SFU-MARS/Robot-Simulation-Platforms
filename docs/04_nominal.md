# 04 Nominal controller

代码：[`nav_core/nominal/`](../nav_core/nav_core/nominal/)

## 4.1 定位

nominal controller 只负责**往 goal 走**，**不提供任何安全保证**。避障完全由 safety filter 负责。

统一接口：

```python
w = nominal.control(x, goal, grid)    # x = (x, y, theta)，返回 turn rate (rad/s)
nominal = make_nominal(profile, "pursuit" | "mppi")
```

这样拆分的好处是 filter 的介入次数（intervention rate）可以直接解释：nominal 越好，filter 越不用出手。

> FaST-BRT 自己也有一份 `filter/nominal.py`（pursuit + 3D MPPI），那是论文 E16/E17 实验专用、已经冻结的版本。本 repo 的 nominal 是独立的一份，因为将来 4D / 6D 机器人都要用，而且依赖方向只能是本 repo → FaST-BRT。

## 4.2 Pursuit

代码：[`pursuit.py`](../nav_core/nav_core/nominal/pursuit.py)

最简单的控制器：**始终把车头对准 goal**。

```python
bearing = atan2(goal.y - x.y, goal.x - x.x)      # goal 在哪个方向
err     = wrap_to_pi(bearing - theta)            # 车头偏差
w       = clip(k_gain * err, -w_max, w_max)      # 偏多少转多少，饱和
```

它**完全不看地图**，所以遇到障碍只会直冲过去。作用有两个：

- 作为 baseline，对照 MPPI 好多少；
- 作为 filter 的压力测试：它会不断提出危险命令，正好逼 filter 出手。

## 4.3 MPPI

代码：[`mppi.py`](../nav_core/nav_core/nominal/mppi.py)

Model Predictive Path Integral：采样式 MPC。每个周期：

1. 在上一周期的方案附近**随机采样 N = 256 条**未来 H = 30 步（3 秒）的 turn rate 序列；
2. 每条都在**自己的地图上**前向模拟，算总代价；
3. 按 `exp(-cost / λ)` 加权平均，得到新方案，取第一步执行；
4. 方案整体左移一格作为下一周期的起点（warm start）。

默认参数：`H=30, dt=0.1, N=256, sigma=0.6*w_max, lam=0.05`。

### 代价函数

```python
cost += 0.1 * c2g                                  # 每一步：离 goal 还有多远
cost += 1e3 * (d <= footprint_radius)              # 撞上了：重罚
cost += 0.5 * exp(-(d - footprint_radius) / 0.03)  # 贴太近：软罚
cost += 2.0 * c2g_terminal                         # 终点：离 goal 还有多远
```

- `d` 是到最近 occupied 格子的距离（`distance_transform_edt`）；
- `c2g` 是**cost-to-go**，下面细说。

### CostMap：geodesic cost-to-go

这是 MPPI 能绕开障碍的关键。如果用**直线距离**当 cost-to-go，机器人会一直被拉向 goal 的方向，撞在墙上出不来（局部极小）。

`CostMap` 改用**绕开障碍的最短路径长度**：

1. 把地图按 footprint 膨胀，得到可通行格子；
2. 在可通行格子上建 8-邻接图（对角权重 √2），边权是实际距离；
3. 从 goal 跑一次 Dijkstra，得到每个格子到 goal 的**测地距离**。

```python
cm = CostMap(grid, goal, footprint_radius)
c2g, d = cm.lookup(points)      # (测地 cost-to-go, 到障碍的距离)
```

自由空间里它退化成 octile 距离；有墙挡着时它会自动绕过去。实测：

| 位置 | 直线距离 | geodesic c2g |
| --- | --- | --- |
| 起点 (−0.8,−0.8) → goal (0.7,0.2) | 1.803 m | 1.914 m（要绕一下 obstacle 2） |
| 一面墙挡在中间的测试 | 0.6 m | > 1.2 m（绕墙端走） |

**unknown 空间算可通行**，所以 MPPI 是乐观的：它会主动往没看过的地方探索。安全由 filter 兜底。

CostMap 带缓存：只有 goal 或地图的 occupied 格子变化时才重算 Dijkstra（`_costmap` 里用 `(goal, occupied.tobytes())` 当 key）。

### 效果

2026-11-22 用迁移后的 backup filter、默认 `V₀`（线性 / low）和内移放置重跑（`experiments/filter_batch.py`，unknown occupied）：

| 场景 | 控制器 | 不加 filter | 加 backup filter |
| --- | --- | --- | --- |
| `hj_reach_target`（27 次） | pursuit | 7 到达，**20 撞** | 16 到达，0 撞，11 超时，介入率 0.24 |
| | **MPPI** | 20 到达，**7 撞** | 16 到达，0 撞，11 超时，介入率 **0.049** |
| 柱子场景（28 次） | pursuit | 19 到达，**9 撞** | 28 到达，介入率 0.057 |
| | **MPPI** | 28 到达，0 撞 | 28 到达，介入率 **0.004**，最小距离 4.4 cm |

两个结论：

- **MPPI 明显更好**：不加 filter 时碰撞大幅减少；加上 filter 后介入率低得多（hj 场景 0.049 对 0.24，柱子场景 0.004 对 0.057）。
- **MPPI 自己仍然会撞**（hj 场景 7 次），所以 filter 仍然必要。

### 超时与 nominal 无关

`hj_reach_target` 里 11 次超时，换成 MPPI 也没减少。原因已经查清：那个场景左上角只能通过 obstacle 1 和北墙之间一条 **0.4 m 宽**的缝进去，而 loiter circle 需要约 0.45 m，filter 永远不放机器人进去。27 次实验里恰好有 11 次需要过这条缝，数字完全对上。这是 constant-speed 模型的结构性限制，不是 nominal 的问题（见 [05](05_safety_filter.md) §5.6）。

## 4.4 扩展到其它 dynamics

MPPI 的动力学只通过 `step` 进入：

```python
MPPI(v, w_max, footprint_radius, step=my_step_fn)
# step(z, u, dt) -> z_next，z 是 (N, state_dim) 的批量状态
```

所以 QBot（4D）和 Crazyflie（6D）可以直接传各自的 forward model（建议从 FaST-BRT import，保证和 filter 用的是同一个模型）。代价函数里只用到了位置，其余部分不用改。

`make_nominal` 目前对非 `dubins3d` 的 dynamics 直接抛 `NotImplementedError`，等模型到位再放开。
