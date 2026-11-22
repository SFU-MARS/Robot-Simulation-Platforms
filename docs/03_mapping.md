# 03 Mapping：从 lidar 到障碍

代码：[`nav_core/mapping/`](../nav_core/nav_core/mapping/)

这一层把 lidar 读数变成三样东西，依次是：

```
scan ──► ① OccupancyGrid          每个格子：未知 / 自由 / 障碍
           │
           ├──► ② free_region      机器人可用区域（给 safety）
           └──► ③ extract_obstacles 障碍的 face、长度、corner（给 primitive）
                     │
                     └──► ④ ObstacleTracker  稳定 ID + 变化事件
```

## 3.0 数学表述（总览）

后面 3.1–3.4 是各步的实现细节，这里先给出整个检测算法的数学形式。

### 记号

| 符号 | 含义 |
| --- | --- |
| $x_k = (p_k, \theta_k)$ | 第 k 个周期的机器人位姿，$p_k \in \mathbb{R}^2$ |
| $Z_k = \{(\varphi_j, r_j)\}_{j=1}^{m}$ | 一帧 2D 扫描：射线相对车头的角度和读数 |
| $R,\ r_{\min}$ | 软件截断的感知距离、传感器最小量程 |
| $u(\phi) = (\cos\phi, \sin\phi)$ | 单位方向 |
| $\mathcal{C}$，$\delta$，$o_c$ | 网格单元集合、单元边长、单元 c 的中心 |
| $c(q)$ | 点 q 所在的单元 |
| $r_{\text{fp}}$ | 机器人 footprint 半径 |

### (1) 射线模型

有效射线：$\mathcal{J}_k = \{ j : r_j \ge r_{\min},\ r_j \ne \text{NaN} \}$（REP 117：`nan`、`-inf`、过近的读数整条丢弃）。

有效长度 $\rho_j = \min(r_j, R)$，射线方向 $u_j = u(\theta_k + \varphi_j)$。

$$
H_k = \{\, c(p_k + r_j u_j) : j \in \mathcal{J}_k,\ r_j < R \,\}
\qquad\text{（命中单元）}
$$

$$
P_k = \{\, c(p_k + t\,u_j) : j \in \mathcal{J}_k,\ 0 \le t < \rho_j - \delta/2 \,\} \setminus H_k
\qquad\text{（穿过单元）}
$$

**擦边保护**：已有命中记录（$h_c > 0$）的单元 c 留在 $P_k$ 里，当且仅当存在某条射线 j 经过 c，且 $o_c$ 到该射线所在直线的距离 $\le \gamma\delta$（$\gamma = 0.3$）。

### (2) 计数更新与单元状态

$$
h_c \leftarrow h_c + \mathbb{1}[c \in H_k], \qquad m_c \leftarrow m_c + \mathbb{1}[c \in P_k]
$$

$$
s(c) =
\begin{cases}
\text{OCC}  & h_c + m_c > 0 \ \text{且}\ h_c \ge \tau\,(h_c + m_c) \\
\text{FREE} & h_c + m_c > 0 \ \text{且}\ h_c < \tau\,(h_c + m_c) \\
\text{UNK}  & h_c + m_c = 0
\end{cases}
\qquad \tau = 0.2
$$

$h_c / (h_c + m_c)$ 是"射线在 c 里终止的频率"的估计（reflection map / counting model）。表面单元部分被占据，这个频率远小于 1/2，所以阈值取 0.2。

### (3) 机器人可用区域（configuration space）

$$
F = \{c : s(c) = \text{FREE}\}
\quad\text{（`unknown="free"` 时取 } F = \{c : s(c) \ne \text{OCC}\}\text{）}
$$

$$
F_r = \{ c \in F : \operatorname{dist}(o_c,\ \mathcal{C}\setminus F) > r_{\text{fp}} \}
$$

地图外的单元也算在 $\mathcal{C}\setminus F$ 里。区域 $\Omega_k$ 是 $F_r$ 中包含 $p_k$ 的连通分量，边界 $\partial\Omega_k$ 取指示函数 $\mathbb{1}_{F_r}$ 的 1/2 等值线（marching squares），简化成多边形：一条 outer ring 加若干 inner ring（被自由空间包围的障碍）。

### (4) 障碍 = 占据单元的连通分量

$$
O = \{c : s(c) = \text{OCC}\}, \qquad
O_1, \dots, O_n = \text{8-连通分量}\big(\operatorname{dilate}_1(O)\big) \cap O
$$

先膨胀 1 个单元再求连通，避免一两个没扫到的单元把同一面墙拆开。互相接触的障碍属于同一个 $O_i$。

### (5) 面（face）：包裹直线

设 $\Gamma$ 是 $\mathbb{1}_F$ 的 1/2 等值线（自由空间的边界），给 $\Gamma$ 上每个点 q 一个标签：

$$
\ell(q) = i \ \text{若离 q 最近的非自由单元属于 } O_i,\qquad \ell(q) = 0 \ \text{（未知 / 地图外）}
$$

把 $\Gamma$ 切成标签恒定的最大段（夹在同一 $O_i$ 中间、不超过 g = 4 个顶点的小段改标为 i），对每个 $\ell = i > 0$ 的段：

1. **折线拟合**：Douglas–Peucker（容差 $\varepsilon = 1.5\delta$），补回凸角，再删掉转角 $< \psi_{\min} = 20^\circ$ 且合并后所有点偏离 $\le 2\varepsilon$ 的顶点；
2. **包裹平移**：对折线的每条边 $[v_a, v_b]$，取指向自由空间的单位法向 $n$，以及分配给它（最近边）的边界点集合 $Q$：

$$
s = \max\Big(0,\ \max_{q \in Q} (q - v_a)\cdot n\Big) + \text{offset}, \qquad
\text{直线 } \{\, x : (x - v_a)\cdot n = s \,\}
$$

3. **端点**：相邻两条平移后直线的交点（转角 $\ge \psi_{\min}$ 时；否则取中点），开放段的两端沿方向各延长 offset。

得到 face $f = (a, b, n, i)$，长度 $L_f = \|b - a\|$。**包裹性**：对所有 $q \in Q$ 有 $(q - a)\cdot n \le s$，即该段所有 detected 边界点都在直线的障碍一侧。`offset = 0` 给出真实边长；`offset = r_fp` 给出 configuration-space 的面。

### (6) 拐角

同一段内相邻的两个 face $f_k, f_{k+1}$，方向 $d_k = (b_k - a_k)/L_k$：

$$
\text{corner} = a_{k+1}, \qquad
\psi = \arccos(d_k \cdot d_{k+1}), \qquad
\text{convex} \iff d_{k+1} \cdot n_k < 0
$$

convex corner 正是 primitive 组合会 leak 的地方。

### (7) 跨周期追踪

设 $L_k(c)$ 是本周期 (4) 给出的原始编号，$S_{k-1}(c)$ 是上一周期的稳定编号：

$$
\operatorname{ov}(i, s) = \big|\{ c : L_k(c) = i,\ S_{k-1}(c) = s \}\big|, \qquad
\operatorname{owner}(s) = \arg\max_i \operatorname{ov}(i, s)
$$

分量 i 继承 $\min\{ s : \operatorname{owner}(s) = i \}$（最早的编号）；若有多个 s 指向 i，记为 merged；没有则分配新编号（new）。face 数量、凸/凹角数量变化，或某个 face 端点移动超过 $2\delta$，记为 updated。少于 4 个单元的分量照常追踪，但不报告。

### (8) face → primitive

对 configuration-space 的 face $(a, b, n)$：

$$
L = \|b - a\|, \quad \beta = \operatorname{atan2}(b - a), \quad
c = \frac{a + b}{2} - n\,\frac{L}{2\sqrt{\alpha}}
$$

$$
V_f^s(x) = V_0^s\big(g^{-1} x\big) - \frac{L^2}{4}, \qquad g = (c, \beta)
$$

中心沿法向**往障碍内侧挪半个厚度**，让矩形朝向自由空间的那条边正好落在 face 上：厚度 $L/\sqrt{\alpha}$ 全部在障碍内部，BRT 朝自由空间一侧的范围只由长度 L 决定。

> **当前代码还不是这样**：现在用的是以 face 为中线的对称放置（$c = (a+b)/2$），靠把 face 切成 ≤ 0.179 m 的小段来限制往自由空间的伸出。这是需要修正的部署偏差，见 [05 §5.3](05_safety_filter.md)。

## 3.1 OccupancyGrid：累积地图

代码：[`occupancy.py`](../nav_core/nav_core/mapping/occupancy.py)

地图是固定分辨率的网格（Burger 是 2 cm 一格，105×105）。仿真和 Gazebo 里 pose 都是已知的（odometry 或 ground truth），所以**不需要 SLAM**，做 known-pose mapping 就行。

### 计数模型（counting model）

每个格子记两个数：

- `hits`：有射线**终止**在这个格子（光被挡住了）；
- `passes`：有射线**穿过**这个格子继续往前。

判定：

```python
occupied  if  hits / (hits + passes) >= tau      # tau = 0.2
free      if  被观测过但比例低于 tau
unknown   if  从未被观测过
```

**为什么不用经典的 log-odds？** log-odds 是加减分：打中 +0.85，穿过 −0.7，分数上下限 ±4。问题出在障碍表面的格子上：

一个 2 cm 的格子压在障碍表面时，**一部分是障碍、一部分是空的**。擦着空的那部分飞过去的射线远多于正好打中的射线，于是：

- log-odds：打中一次（+0.85）、擦过一次（−0.7）= 0.15，低于阈值 0.6，格子**退回 unknown**。face 和 corner 就开始闪。
- 计数模型：20% 的命中率就足以判为障碍，哪怕被擦过 20 次、只命中 5 次，它也稳定是障碍。
- 空地上的噪声假命中（1 次命中、50 次穿过 = 2%）仍然判为自由。
- 计数**不截断**，历史证据不会被遗忘；log-odds 封顶 ±4，一个被打中很多次的格子也只记到 4，之后被擦 6 次就清零了。

### 擦边保护 `graze`

再加一层：**已经有命中证据的格子，只有射线从它中心附近（0.3 个格子以内）穿过才算 `passes`**，只蹭到角的不算。

### 效果

遮挡场景、3 条路线，统计"障碍格子数回落"的次数（静态场景里障碍只应变多，回落基本就是抖动）：

| 地图模型 | 无噪声 | 1 cm 噪声 | 2 cm 噪声 | face 最深切进真实障碍（1 / 2 cm 噪声） |
| --- | --- | --- | --- | --- |
| log-odds（旧默认） | 38 | 80 | 112 | 3.0 / 2.0 cm |
| log-odds + graze | 25 | 61 | 76 | 3.0 / 1.5 cm |
| **counts τ=0.2 + graze（现默认）** | **2** | **15** | **27** | **0.3 / 0.6 cm** |

旧模型仍然保留：`OccupancyGrid(..., model="logodds")`。

### scan 读数的约定（REP 117）

`integrate_scan` 严格按 ROS 的 `sensor_msgs/LaserScan` 规范处理：

| 读数 | 含义 | 处理 |
| --- | --- | --- |
| 有限值，在 `[range_min, range)` 内 | 打到东西 | 沿途 `passes`，终点 `hits` |
| `+inf` 或 ≥ 量程 | 射程内没有回波 | 沿途到量程全部 `passes`，无 `hits` |
| `nan`、`-inf`、小于 `range_min` | **无效读数** | **整条射线跳过** |

最后一条很关键。无效读数曾被当成"没有回波"，于是整条射线被标成自由，**把已经建好的障碍擦掉**。而障碍贴得太近时，真实 lidar 返回的恰恰就是这类无效值。

## 3.2 free_region：机器人可用区域

代码：[`boundary.py`](../nav_core/nav_core/mapping/boundary.py) 的 `free_region`

```python
outer, holes = free_region(grid, xy, footprint_radius=0.105, unknown_free=False)
```

做三件事：

1. **按 footprint 收缩**（`safe_free_mask`）：距离任何非自由格子小于 `footprint_radius` 的格子被剔除。收缩之后**机器人就可以当成一个点**，后面所有几何都在 configuration space 里。
2. **提取边界**：marching squares（`contourpy`）取等值线，Douglas–Peucker 简化成折线，再 `snap_corners` 把凸角补回来（marching squares 会把每个凸角削掉一小段，不补的话长度要少估约 2 个 cell）。
3. **区分内外**：返回包含机器人的那块区域的 **outer ring**（逆时针），以及它内部所有孤立障碍的 **inner rings**（顺时针）。

### 为什么要 inner rings

一根柱子被机器人绕着看了一圈之后，在地图上就成了自由区域中间的一个"洞"：

```
┌──────────────────┐
│      free        │
│      ┌──┐        │
│      │██│ ← 柱子（实心）
│      └──┘        │
│      free        │
└──────────────────┘
自由区域 = 外边界以内 减去柱子 → 像一个"环"
```

FaST-BRT 原来的 `SceneModel` 只接受**一个简单多边形**（论文里用的是单帧 visibility polygon，从机器人往外看是星形，永远没有洞）。如果只把 outer ring 交给它，柱子就落在多边形内部，会被当成自由空间，**这是不安全的**。

所以 `free_region` 返回 outer + holes，`RegionScene` 对多条 ring 都处理（见 [05](05_safety_filter.md)）。

### `unknown_free` 开关

| | region 的含义 |
| --- | --- |
| `unknown_free=False`（`unknown="occupied"`） | 已知自由的格子；unknown 在区域外，frontier 是边界 |
| `unknown_free=True`（`unknown="free"`） | 所有非 detected-occupied 的格子；每个检测到的障碍各自围成一个 ring |

两种模式下**区域之外一律是障碍**。所以不管障碍是一整根柱子，还是只看到一个面、只有一两层格子厚的一小块，按 footprint 外扩后都会形成一个闭合 ring，**ring 内部整块算障碍**。detected 的每个格子都被完整覆盖，而不是只覆盖轮廓。

## 3.3 extract_obstacles：障碍层

代码：[`obstacles.py`](../nav_core/nav_core/mapping/obstacles.py)

```python
om = extract_obstacles(grid, offset=0.0)
om.n          # 障碍个数
om.faces      # [Face(obstacle, a, b, normal), ...]，face.length 是边长估计
om.corners    # [Corner(obstacle, faces, point, convex, turn), ...]
```

流程：

1. **`label_obstacles`**：occupied 格子做 8-连通的 connected component。先膨胀 1 个 cell 再求连通，避免一两个没扫到的格子把一面墙拆成两段。
2. **沿边界走**：取"自由空间 / 非自由空间"的等值线，用最近邻判断每个边界点背后是哪个障碍，把等值线按障碍切成若干 run。夹在同一障碍中间、不超过 4 个顶点的 frontier 小段会被补成这个障碍（`_fill_short_gaps`）。
3. **拟合 face**（`_run_faces`）：
   - Douglas–Peucker 简化 + `snap_corners`；
   - `_merge_shallow`：转角小于 20° 且合并后所有边界点偏离不超过 2×tol 的顶点被删掉。噪声会把直墙搞得锯齿状，这一步把它拉直；真正的曲线会保留顶点。这解决了"1 cm 噪声下墙碎成好几段"的问题。
   - **包裹（enclosing）**：每条直线沿法向往自由侧平移，平移量取"它负责的那段边界点里偏离最大的那个"。这样**该段所有 detected 边界点都落在直线的障碍一侧**，这就是"包裹"的含义。
   - 相邻两条直线求交得到顶点；开放 run 的两端按 `offset` 外延。
4. **corner**：相邻 face 的交点，按法向判断 convex / concave，记下转角大小。

### offset 参数

`offset > 0`（通常传 `footprint_radius`）时，face 再往外推 `offset`，两端也各延长 `offset`。得到的就是 **configuration-space 障碍**，只是拐角是方的而不是圆的（更保守，安全方向）。

| 调用 | 用途 |
| --- | --- |
| `extract_obstacles(grid)` | 估计**真实边长**，发给 leaking-corner / 日志 |
| `extract_obstacles(grid, offset=footprint_radius)` | 生成 **primitive** 给 safety filter |

### 精度

`hj_reach_target` 无噪声，机器人走完一圈后：

- 1 个障碍（两个 obstacle 和西墙连在一起，按设计决策算一个），12 个 face；
- **4 个 convex corner 正好是 obstacle 1 和 2 的 4 个外角**；
- obstacle 1 的四条边估成 0.39 / 0.42 / 0.50 / 0.53 m（真实 0.4 × 0.5 m）；
- 所有 face 都在真实障碍外侧至少 0.95 cm（包裹成立）；
- 1 cm 噪声、每处扫 10 帧：13 个 face、5 个 convex corner，包裹仍然成立（face 在障碍外侧至少 0.86 cm）。换计数地图之前，噪声下 face 会切进真实障碍约 0.6 cm，靠 3 cm 的 clearance 兜住。

### 连在一起的障碍算一个

这是明确的设计决策。`hj_reach_target` 里 obstacle 1、obstacle 2 和西墙互相接触，会被当成**一个**障碍。face 和 corner 仍然是对的，只是 ID 合并了。以后需要的话可以在凹角处切开。

## 3.4 ObstacleTracker：稳定 ID 与检测事件

代码：[`tracking.py`](../nav_core/nav_core/mapping/tracking.py)

`extract_obstacles` 每个周期都从头重新标号，所以它给的 ID **不稳定**：

> 第 1 秒：墙是 1 号，箱子是 2 号
> 第 2 秒：又看到一根柱子，按栅格顺序重排，柱子变成 1 号、墙变成 2 号

于是没法说"2 号箱子的长度从 0.1 m 变成了 0.2 m"。

```python
om, events = tracker.update(extract_obstacles(grid), t, grid.res)
```

**匹配方式**：occupied 格子基本只增不减（障碍越看越全），所以按**共享格子数**匹配。这一刻的某个障碍如果和上一刻某个 ID 占了相同的格子，就沿用那个 ID。这样就不需要跨帧的 data association。

**事件类型**：

| kind | 含义 |
| --- | --- |
| `new` | 第一次报告某个障碍 |
| `updated` | face 数量或 corner 数量变了，或某个 face 端点移动超过 2 个 cell（通常是看到了更多） |
| `merged` | 两个已报告的障碍连在一起，保留较早的 ID，`merged_from` 列出被合并的 |
| `split` | 一个部分失去了原 ID（噪声） |
| `removed` | 某个 ID 的 occupied 格子全没了（噪声） |

**噪声抑制**：小于 4 个格子的碎片（擦边的零星命中）会被追踪但不报告，等它长大或合并时再报。

每条事件带着该障碍的 face（含长度）、corner、格子数，`to_dict()` 可直接序列化成 JSON。

**接在三个地方**：

| 位置 | 用途 |
| --- | --- |
| `run_episode(..., tracker=...)` | 事件存进 `Episode.events`，`run_sim.py --events` 打印 |
| `FastBrtGatekeeper` | 它用的 face 带稳定 ID，本周期事件在 `gate.events` |
| ROS 节点 | 每条事件以 JSON 发到 `obstacle_events` topic |

最后一条就是给师妹的接口：订阅 `obstacle_events`，只对有变化的障碍做 leaking-corner update，不用每周期全量重算。

### 实际例子（遮挡场景）

```
t=0.0s   obstacle 3 new:     一面墙，看到 0.91 m
t=0.2s   obstacle 3 updated: 0.95 m
  ……（机器人沿墙前进，看到的越来越多）
t=4.6s   obstacle 3 updated: 1.40 m          ← 真实长度 1.4 m
t=8.6s   obstacle 4 new:     藏在墙后的箱子，先看到 0.10 m
t=10.4s  obstacle 4 updated: 两面 0.23 / 0.20 m，1 个凸角   ← 真实 0.2 × 0.2 m
```

这正是"检测到障碍 → 加进地图 → 快速估计长度"这条任务链，而且能看到估计随观测单调变准的过程。
