# 07 MPPI 完整案例：从加载 map 到 2D primitive

这一篇把**一个 control period** 从头到尾走一遍，每步都带真实的运行数字。

**场景**：Burger 在 `hj_reach_target.world` 里，从 `(-0.8, -0.8, 0.2 rad)` 出发去 `(0.7, 0.2)`，nominal 用 MPPI，safety 用 FaST-BRT gatekeeper。

```
      y
  1.0 ┌───────────────────────┐     ███ obstacle（红）
      │        ███            │      ★  goal
      │     ███████           │      ●  机器人起点
  0.5 │     ███████           │
      │     ███████  ★        │     obstacle 1: x∈[-0.1,0.3], y∈[0.1,0.6]
  0.0 ├──────────             │     obstacle 2: x∈[-1.0,-0.1], y∈[-0.3,0.1]
      │██████████             │     goal:       (0.7, 0.2)
 -0.5 │                       │
      │   ●                   │
 -1.0 └───────────────────────┘
     -1.0        0.0        1.0  x
```

全文的数字都可以用文末的脚本复现。

---

## Step 0：加载 map 和 profile

### 0.1 Robot profile

```python
from nav_core.robots import load_profile
p = load_profile("burger").require_ready()
```

```
v 0.1 m/s   w_max 1.0 rad/s   转弯半径 R = v/w_max = 0.1 m   footprint 0.105 m
轮速检查：0.1 + 1.0 × 0.16/2 = 0.18 m/s ≤ 0.22 m/s          ✓
Units：    1 模型长度单位 = 0.1 m，1 模型时间单位 = 1.0 s
Map：      origin (-1.05, -1.05)，size 2.1 × 2.1 m，res 0.02 m → 105 × 105 格
```

### 0.2 加载 world（真实障碍，仅仿真用）

```python
from nav_core.sim import World
w = World.from_sdf(ROBOTS_DIR.parent / p.world, plane_z=p.sensor.height)
```

从 SDF 里取出所有穿过扫描平面（0.17 m）的 collision box：

```
6 boxes：
  center [0.000,  1.025]  half [1.025, 0.025]   北墙
  center [0.000, -1.025]  half [1.025, 0.025]   南墙
  center [-1.025, 0.000]  half [0.025, 1.000]   西墙
  center [1.025,  0.000]  half [0.025, 1.000]   东墙
  center [0.100,  0.350]  half [0.200, 0.250]   obstacle 1 (0.4 × 0.5 m)
  center [-0.550,-0.100]  half [0.450, 0.200]   obstacle 2 (0.9 × 0.4 m)
```

地板和 goal / start 贴纸自动被过滤掉（不跨扫描平面）。

**注意**：这个 `world` 只是仿真用的 ground truth，**机器人看不到它**，只能通过 lidar 间接感知。上 Gazebo 时这一步换成 Gazebo 自己。

### 0.3 机器人自己的地图（一开始全是未知）

```python
from nav_core.mapping import OccupancyGrid
grid = OccupancyGrid.from_profile(p)
# model='counts'  tau=0.2  graze=0.3  shape=(105, 105)
```

全部 11025 个格子都是 `unknown`。

---

## Step 1：lidar 扫描

```python
angles = np.arange(360) * 2π / 360
ranges = w.raycast(x[:2], x[2] + angles, 0.8)      # 仿真；ROS 里是 /scan 回调
```

```
360 条射线：307 条在 0.8 m 内打到东西，53 条返回 +inf（射程内无回波）
最近 0.200 m（西墙），最远有限值 0.797 m
```

---

## Step 2：更新地图

```python
grid.integrate_scan(x, angles, ranges, range_max=0.8, range_min=0.12)
```

对每条射线：沿途格子 `passes += 1`，终点格子 `hits += 1`（已被命中过的格子有擦边保护）。

```
这一帧：118 个格子被命中，1607 个格子被穿过
地图状态：free 1607   occupied 118   unknown 9300
```

判定规则 `hits / (hits + passes) ≥ 0.2`。

---

## Step 3：提取障碍（face、长度、corner）

```python
om = extract_obstacles(grid)         # offset=0，估真实长度
```

```
1 个 obstacle，3 个 face，2 个 corner：
  face  a=[-0.20,-0.31]  b=[-0.98,-0.31]  L=0.780 m  normal=[ 0,-1]   obstacle 2 的南面
  face  a=[-0.98,-0.31]  b=[-0.99,-0.98]  L=0.670 m  normal=[ 1, 0]   西墙
  face  a=[-0.99,-0.98]  b=[-0.03,-0.98]  L=0.960 m  normal=[ 0, 1]   南墙
  corner [-0.98,-0.31]  concave      obstacle 2 和西墙的接缝
  corner [-0.99,-0.98]  concave      西南墙角
```

为什么只有 1 个 obstacle：obstacle 2 贴着西墙，西墙又接着南墙，connected component 把它们算成一个（设计决策）。face 和 corner 仍然是对的。

obstacle 2 的南面真实长度是 0.9 m，这里估成 0.78 m，因为**右端还没被看到**（被自己挡住了）。随着机器人前进，这个估计会单调变长到 0.9 m。

给 filter 用的版本要外推 footprint：

```python
omf = extract_obstacles(grid, offset=0.105)    # face 长度变成 [0.782, 0.46, 0.958]
```

（中间那条短了，因为两端都是凹角，外推后交点内缩。）

---

## Step 4：追踪，发检测事件

```python
om, events = tracker.update(om, t=0.0, res=0.02)
```

```
event: obstacle 1 new: 118 cells, faces [0.960, 0.780, 0.670] m, 0 convex / 2 concave corners
```

第一个周期是 `new`。往后只有变化时才发，例如 t=2.0 s：

```
event: obstacle 1 updated: 137 cells, faces [1.110, 0.890, 0.660] m, 0 convex / 2 concave corners
```

ROS 里这条事件会以 JSON 发到 `obstacle_events` topic。

---

## Step 5：MPPI 给出 nominal 命令

```python
w_nom = nominal.control(x, goal, grid)
```

### 5.1 先建 CostMap

```python
cm = CostMap(grid, goal, footprint_radius=0.105)
```

- `obst`：到最近 occupied 格子的距离；
- `c2g`：从 goal 出发，在"按 footprint 膨胀后仍可通行"的格子上跑一次 Dijkstra，得到**绕开障碍的最短路径长度**。

```
起点处：geodesic c2g = 1.914 m   （直线距离只有 1.803 m，差值就是绕 obstacle 2 的代价）
        到障碍距离 d = 0.200 m
```

unknown 格子算**可通行**，所以 MPPI 愿意往没看过的地方探索。

### 5.2 采样、模拟、加权

```
H = 30 步 × dt = 0.1 s = 3 秒预测
N = 256 条采样，sigma = 0.6 × w_max，lam = 0.05
```

每条采样序列前向模拟 30 步，累加代价：

```python
cost += 0.1 * c2g                                  # 离 goal 多远（绕障碍的距离）
cost += 1e3 * (d <= 0.105)                         # 撞了：重罚
cost += 0.5 * exp(-(d - 0.105) / 0.03)             # 贴太近：软罚
cost += 2.0 * c2g_terminal                         # 终点再罚一次
```

按 `exp(-(cost - cost_min) / lam)` 归一化加权平均，取第一步：

```
w_nom = +0.2823 rad/s      （轻微左转，朝 goal 方向，同时避开左边的 obstacle 2）
```

---

## Step 6：Safety filter

```python
ws = gate(x, w_nom, grid)
```

### 6.1 离线准备（只在构造 gatekeeper 时做一次）

```python
gate = FastBrtGatekeeper(p)       # alpha=20, horizon=2.0, sub_dt=0.05
```

```
BaseValue 求解一次：grid 121 × 121 × 60，41 个时间切片，horizon 2.0   耗时 1.04 s
clearance    = 1.5 cell = 0.03 m = 0.3 模型单位
max_side_len = 2 × h_max × √alpha = 2 × 0.2 × √20 = 1.789 模型单位 = 0.179 m
n_sub        = ctrl_dt / sub_dt = 0.2 / 0.05 = 4
```

**这是全流程里唯一一次解 HJ**，之后所有 primitive 都靠平移查表。

### 6.2 构造 scene：region

```python
outer, holes = free_region(grid, x[:2], footprint_radius=0.105)
```

```
outer ring 5 个顶点，0 个 hole：
  [-0.89, -0.88]  [-0.12, -0.89]  [-0.11, -0.70]  [-0.21, -0.42]  [-0.88, -0.41]
```

这块五边形就是**机器人现在可以安全使用的全部空间**。它比扫描看到的自由空间小一圈（按 0.105 m 收缩），所以后面可以把机器人当成一个点。区域之外（包括所有 unknown）一律算障碍。

### 6.3 构造 scene：2D primitive ★

障碍层的 face（已外推 footprint）被切成不超过 `max_side_len` 的小段，每段生成一个 primitive：

```python
side_frame(a, b, alpha=20)
  → Rect(center=(a+b)/2, half_len=L/2, half_wid=L/(2√20), angle=β)
```

```
18 个 primitive
  L（模型单位）    0.951 ~ 1.649      → 0.095 ~ 0.165 m
  半宽（模型单位）  0.106 ~ 0.184      → 0.011 ~ 0.018 m（即 ≤ 1 个 cell）
```

**每个 primitive 的 value 不用求解**，而是把查询点变换到标准坐标系后查 `V₀` 的表，再减一个常数：

```
V_i^s(x) = V₀^s( g_i⁻¹ x ) − q_L ,      q_L = L²/4,   g_i = (center_i, β_i)
```

所有 primitive 取 min 就是组合 value：

```python
U(x) = min_i V_i(x)       # 机器人当前位置：U = 10.66 > 0
```

`U > 0` 表示**没有任何单个 primitive 判定它注定要撞**。注意 `U` 只用符号，数值本身是 quadratic encoding 下的量，不是距离。

为什么要切短：primitive 的**厚度和长度绑定**（半宽 = `L/(2√α)`）。一条 1.35 m 的墙对应的矩形会往自由空间里伸出 15 cm，在 2 m 的场地里会把通道堵死。切到 0.179 m 以内，伸出就不超过 1 个 cell。

切短**不影响安全**：witness 的碰撞判据是 region 的 signed distance，不是 primitive 矩形。

```
机器人位置的 margin = 0.5714 模型单位 = 0.0571 m
（离 region 边界还有 5.7 cm，远大于 clearance 的 3 cm）
```

### 6.4 第一个周期：先给自己找一条后路

```python
gate.commit_initial(xm, model)      # 找不到就 RuntimeError，机器人不出发
```

### 6.5 模拟 nominal 的这 0.2 秒

把 `w_nom` 换算到模型单位，用 `step_arc` 走 4 个 sub-step，记录全程最低 margin。

### 6.6 从终点找 witness

```python
find_witness(x_next, model, dt=0.05)
```

```
horizon 2.0 模型时间 → K = 40 步
候选：1 条 HJ switching + 17 条几何候选（左打死 / 右打死 / 直行 / 先直后打）
      一次性批量 rollout

结果：FOUND，loiter 方向 = 左
      前 8 个控制 [-1, -1, +1, -1, +1, -1, +1, -1]   ← HJ switching policy（闭环，所以来回切换）
```

验证通过的含义：这条轨迹全程 `margin > clearance`，而且终点的左侧 loiter circle（半径 0.1 m，72 个采样点）整圈都在 region 内。

### 6.7 放行

```
GATE out = [0.282, 0.282, 0.282, 0.282] rad/s      intervened = False
```

nominal 的命令被原样放行，同时新 witness 被存下来，作为下一个周期的后路。

---

## Step 7：执行

```python
for w in ws:
    x = unicycle_step(x, v=0.1, w, dt=0.05)        # 精确圆弧
```

ROS 里这一步换成按 sub-period timer 发 `/cmd_vel`。

---

## 第二个例子：filter 出手

把 nominal 换成 pursuit（它不看地图），同样的起点，第 29 个周期（t = 5.8 s）：

```
pose (-0.313, -0.488)，heading 34°，离最近障碍 0.083 m（已扣车身）

① pursuit 说：车头已对准 goal，直走        w_nom = 0.00 rad/s
② 模拟这 0.2 秒：终点 (-0.296, -0.477)，全程最低 margin 4.0 cm > clearance 3.0 cm
                 → 这 0.2 秒本身不会撞
③ 从终点找 witness：NONE
                 → 到了那个位置，离 obstacle 2 的拐角太近，机器人又停不下来，
                   往哪边绕圈都会出 region
④ 拒绝，改用上周期存下的 witness：
   GATE out = [-1, -1, -1, -1] rad/s   intervened = True   loiter 方向 = 右
                 → 向右满舵避开拐角
```

这就是 filter 的核心：**它不是等快撞了才躲**（恒速机器人也躲不掉），而是每一步都提前确认"走完这一步之后还有后路"。

---

## 复现脚本

```bash
cd nav_core
PYTHONPATH=.:<FaST-BRT 路径> python - <<'EOF'
import numpy as np
from nav_core.robots import ROBOTS_DIR, load_profile
from nav_core.sim import World
from nav_core.mapping import OccupancyGrid, extract_obstacles, free_region, ObstacleTracker, FREE, OCCUPIED, UNKNOWN
from nav_core.nominal import make_nominal, CostMap
from nav_core.safety import FastBrtGatekeeper
np.set_printoptions(precision=3, suppress=True)

p = load_profile("burger").require_ready(); s, U = p.sensor, p.units
w = World.from_sdf(ROBOTS_DIR.parent / p.world, plane_z=s.height)
grid = OccupancyGrid.from_profile(p); tracker = ObstacleTracker()
x, goal = np.array([-0.8, -0.8, 0.2]), np.array([0.7, 0.2])
angles = np.arange(s.n_rays) * s.fov / s.n_rays

ranges = w.raycast(x[:2], x[2] + angles, s.range)                      # Step 1
grid.integrate_scan(x, angles, ranges, s.range, s.range_min)           # Step 2
st = grid.state()
print("map:", (st == FREE).sum(), "free,", (st == OCCUPIED).sum(), "occ,", (st == UNKNOWN).sum(), "unknown")

om, ev = tracker.update(extract_obstacles(grid), 0.0, grid.res)        # Step 3 + 4
for f in om.faces:  print("face", np.round(f.a, 2), np.round(f.b, 2), round(f.length, 3), "m")
for e in ev:        print(e.summary())

nom, gate = make_nominal(p, "mppi"), FastBrtGatekeeper(p)
cm = CostMap(grid, goal, p.footprint_radius)                           # Step 5
print("c2g", round(float(cm.lookup(x[None, :2])[0][0]), 3), "m vs straight",
      round(float(np.linalg.norm(x[:2] - goal)), 3), "m")
w_nom = nom.control(x, goal, grid); print("w_nom", round(w_nom, 4))

outer, holes = free_region(grid, x[:2], footprint_radius=p.footprint_radius)   # Step 6
print("region", len(outer), "vertices,", len(holes), "holes")
ws = gate(x, w_nom, grid)                                              # Step 6 + 7
m = gate.model
print(len(m.frames), "primitives, L(m) =", np.round([2 * f[1] * U.length for f in m.frames], 3))
print("margin", round(float(m.margin(gate.to_model(x)[None])[0]) * U.length, 4), "m, U =",
      round(float(m.U(gate.to_model(x)[None])[0]), 2))
print("out", np.round(ws, 3), "intervened", gate.interventions)
EOF
```

或者直接看完整运行：

```bash
PYTHONPATH=.:<FaST-BRT 路径> python scripts/run_sim.py --nominal mppi --filter gatekeeper --events
```

---

## 小结：三层各管什么

| 层 | 输入 | 输出 | 决定什么 |
| --- | --- | --- | --- |
| **mapping** | scan + pose | occupancy grid → region / face / corner / event | 机器人"知道"什么 |
| **nominal** | pose + goal + grid | 一个 turn rate | 机器人"想"怎么走 |
| **safety** | pose + turn rate + grid | 4 个 turn rate | 机器人"能"怎么走 |

两条不变量：

1. **安全只依赖 region**（detected 格子以外全算障碍）加 witness 验证。障碍层拟合得差只影响效率。
2. **在线不解 HJ**。所有 primitive 的 value 都由同一个 `V₀` 精确平移得到。
