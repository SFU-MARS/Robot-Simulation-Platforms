# 05 Safety filter：gatekeeper、witness 与 2D primitive

代码：[`nav_core/safety/fastbrt_gatekeeper.py`](../nav_core/nav_core/safety/fastbrt_gatekeeper.py)（adapter）+ FaST-BRT 的 `fast_brt/filter/gatekeeper.py`、`find_witness`（后路搜索）、`fast_brt/primitive.py`

这是整个系统里唯一 import `fast_brt` 的地方。

## 5.1 安全是怎么定义的

机器人恒速前进、**停不下来**。所以"安全"不能定义成"来得及刹车"，只能定义成：

> **永远存在一条可以一直走下去而不碰障碍的路。**

最简单的"一直走下去"是**绕最小圆**（loiter circle）：方向打死，绕半径 R 的圆一直转。于是得到 gatekeeper 的规则：

> 接受 nominal 的命令，当且仅当执行完之后，能找到一条 **witness**：
> 一段 rollout 轨迹，全程保持 clearance，**末端停在一个完全自由的 loiter circle 上**。
>
> 找不到就拒绝，改为执行上一周期已经验证过的 witness。

这保证了 **recursive feasibility**：只要出发时有一条 witness，之后每一步要么接受新命令（同时有了新 witness），要么退回旧 witness。旧 witness 是在旧地图上验证的，而地图只会越来越多自由空间（静态环境），所以旧 witness 始终有效。

**安全不依赖 HJ value 算得准不准。** value 只用来提供候选控制、加速搜索；最终放行与否由 witness 的几何验证决定。

## 5.2 一个周期里 gatekeeper 做什么

`FastBrtGatekeeper.__call__(x, w_nom, grid)`：

```
1. scene(x, grid)              构造这一帧的 scene（region + primitive）
2. 首次调用：commit_initial    在起点先找一条 witness；找不到 → RuntimeError（机器人不出发）
3. 模拟 nominal 的这 0.2 秒    step_arc 走 n_sub 个 sub-step，记录最低 margin
4. gate.filter(...)            从终点找新 witness
      找到 → 放行 w_nom，存下新 witness
      没找到 → 拒绝，返回上一周期存的 witness 的前 n_sub 步
5. 返回 n_sub 个 turn rate（物理单位）
```

`n_sub = ctrl_dt / sub_dt`，Burger 是 `0.2 / 0.05 = 4`。所以每个控制周期输出 4 个 zero-order-hold 的 turn rate。

### witness 搜索（FaST-BRT 的 `find_witness`）

从给定状态出发，按优先级把一批候选控制序列**一次性批量 rollout**：

1. **HJ switching policy**：`ω = w_max · sign(∂V₀/∂θ)`，沿着最紧的那个 primitive 的 value 梯度逃离；
2. 恒定左转 `+1`、恒定右转 `−1`、直行 `0`；
3. "先直行 k 步再打死"的组合（k = 5, 10, 15, …）。

Burger 的配置下 horizon = 2.0 模型时间、`sub_dt = 0.05`，所以 K = 40 步，一共 **17 条几何候选 + 1 条 HJ 候选**。

对每条 rollout：

- 全程 `margin > clearance`（margin 来自 scene，见 §5.4）；
- 终点状态的 **loiter circle**（左右各试一次，72 个采样点）整圈都 `margin > clearance`。

第一条同时满足的就是 witness。都不满足则返回 `None`。

整个搜索是一次批量 rollout，在 laptop 上每步 15–35 ms。

## 5.3 2D primitive：一条边一个，value 靠平移得到

这是 FaST-BRT 的核心，代码在 `fast_brt/primitive.py`。

### 为什么需要它

直接对整个场景解一次 HJ value function 是很贵的，而且地图每个周期都在变。FaST-BRT 的做法是：

> 把障碍拆成一条条**边（facing side）**，每条边对应一个 primitive；
> 所有 primitive 的 value **由同一个 base value `V₀` 平移旋转得到，不用重新求解**。

### 标准 primitive 的形状

长度为 L 的标准 primitive，是一个沿局部 x 轴方向、很薄的矩形：

```
|p_x| ≤ L/2 ,  |p_y| ≤ L / (2√α)
```

`α` 是"细长比"参数，默认 **α = 20**，所以厚度约是长度的 1/4.5。

它的 terminal cost（quadratic encoding，论文用的那个）：

```
ℓ_L(p) = max(p_x² , α p_y²) − L²/4
```

注意 **L 只以常数 `q_L = L²/4` 的形式出现**。这就带来了关键性质：

### 精确平移（exact transport）

对任意 horizon s：

```
长度平移：  V_L^s(p) = V_0^s(p) − q_L           其中 q_L = L²/4
位姿平移：  V^s(x)   = V_0^s(g⁻¹ x) − q_L       g = (center, β) 是这条边的位姿
```

也就是说：**离线只解一次 `V₀`，在线任何一条边的 value 都只是"把查询点变换到标准坐标系 → 查表 → 减一个常数"。** 完全没有在线求解。

代码里：

- `BaseValue.__init__` 解一次 `V₀`，存成 41 个时间切片（s = 0, 0.05, …, 2.0），grid 是 121×121×60，在 laptop 上约 **1 秒**；
- `BaseValue.side_values_many(states, frames, s)` 批量算所有边的 `V_i^s`；
- `U(x) = min_i V_i(x)` 是所有 primitive 的最小值，`U ≤ 0` 表示"至少被一条边判定为注定要撞"。

### side_frame：从一条边到一个 primitive

```python
side_frame(a, b, alpha)  →  Rect(center=(a+b)/2, half_len=L/2, half_wid=L/(2√α), angle=β)
```

把线段 `a→b` 的中点、长度、方向直接变成 primitive 的位姿。

### 长边要切短（当前部署，**需要修正**）

> **这是一个部署偏差。** primitive 的设计意图是：矩形朝向自由空间的那条边和检测到的面重合，厚度 `L/√α` 全部落在障碍内部，这样 BRT 朝自由空间一侧的范围**只由长度 L 决定**，厚度无关紧要，长墙也不用切。
>
> 现在的代码沿用了 FaST-BRT 的 `side_frame`，把矩形**以面为中线对称放置**，于是一半厚度伸进了自由空间，只好再把面切成小段来限制伸出（FaST-BRT 当初选 α=20 加切边，也是为了补偿这一点）。切段还会在共线的接缝处制造额外的 leak。
>
> 修正方法是把中心沿法向往障碍内侧挪半个厚度：`center = (a+b)/2 − n · L/(2√α)`，见 [03 §3.0 (8)](03_mapping.md)。修正后 α 只影响数值精度和"矩形穿出薄障碍背面"的程度，需要单独做一组选 α 的实验。

下面描述的是**当前**的做法：


primitive 的**厚度和长度是绑定的**：半宽 = `L / (2√α)`。一面 1.35 m 长的墙，对应的矩形会往自由空间里伸出 15 cm，在 2 m 的场地里直接把通道堵死。

所以 adapter 用 `max_side_len(h_max, α) = 2 h_max √α` 把边切成小段，让每段伸出不超过 `h_max`：

```
h_max = 1 cell = 0.02 m  →  L_max = 2 × 0.2 × √20 = 1.789 模型单位 = 0.179 m
```

Burger 实测：第一帧 18 个 primitive，长度 0.095–0.165 m，半宽 0.011–0.018 m（即 ≤ 1 cell）。

切开**不影响安全**：witness 的碰撞检查用的是 region 的 signed distance，不是 primitive 矩形（见下一节）。切开只影响 HJ 项的保守程度。

## 5.4 RegionScene：把 region 和 primitive 粘起来

FaST-BRT 的 `SceneModel` 假设场景是**一个简单多边形**（论文里的单帧 visibility polygon）。我们的累积地图里自由区域可能带洞（孤立的柱子），所以 adapter 里写了一个子类 `RegionScene`：

| 接口 | 来源 | 作用 |
| --- | --- | --- |
| `margin(xy)` | **region**：到 outer/inner ring 的 signed distance（区域内为正）；再和 primitive 矩形取 min | witness 的碰撞判据，**决定安全** |
| `side_values` / `U` / `U_s` | **primitive**：`V_0` 平移查表 | HJ 项，提供候选控制、剪枝 |
| `hj_switch(states)` | primitive 的 `∂V₀/∂θ` 符号 | HJ switching 候选 |
| `loiter_side(state, R)` | `margin` | 判断左/右 loiter circle 是否自由 |
| `clearance` | profile 的 1.5 cell | 采样余量 |

**这就是 Design decision 1 在代码里的落点**：安全走 region，primitive 只管效率。

`RegionScene` 按分工应该放到 FaST-BRT 里（在 TODO 里），现在暂放本 repo 以免和师妹的改动冲突。

## 5.5 关键参数

```python
FastBrtGatekeeper(profile,
                  unknown="occupied",   # occupied | free
                  primitives="faces",   # faces | rings
                  alpha=20.0,
                  horizon=2.0,          # 模型时间单位
                  sub_dt=0.05,
                  clearance=None,       # 默认 1.5 cell
                  h_max=None,           # 默认 1 cell
                  search="dive", budget=300)
```

| 参数 | Burger 实际值 | 说明 |
| --- | --- | --- |
| `clearance` | 1.5 cell = 3 cm = 0.3 模型单位 | 覆盖栅格分辨率带来的误差：marching squares 的等值线和格心距离可能比真实自由空间多出约 1 个 cell |
| `h_max` | 1 cell = 2 cm | primitive 伸出边的最大距离 |
| `horizon` | 2.0 模型单位 = 2.0 s | witness 的 rollout 长度，K = 40 步 |
| loiter 半径 R | 1 模型单位 = 0.1 m | 等于转弯半径 |

`primitives` 的两个取值：

- **`"faces"`（默认）**：primitive 来自障碍层的 face（已按 footprint 外推），`unknown="occupied"` 时再加上 frontier 边。同时 `gate.obstacles` / `gate.events` 带稳定 ID，给 leaking-corner 用。
- **`"rings"`**：primitive 直接取 region 每条 ring 的边（早期做法，保留做对比）。

两者安全性相同（都是 0 碰撞），但 `faces` 离障碍更远一些（最小距离 3.4 → 5.5 cm），因为 face 是包裹出来的、拐角是方的。

## 5.6 结构性限制：窄通道进不去

loiter circle 需要的自由空间：

```
   圆直径              2R = 0.2 m
 + 车身两侧            2 × 0.105 m
 + clearance 余量      ≈ 0.04 m
 ≈ 0.45 m 宽
```

`hj_reach_target` 里 obstacle 1 和北墙之间只有 **0.4 m**。机器人车身只有 0.21 m 宽，物理上过得去，但进去之后放不下 loiter circle，后路断了，**gatekeeper 永远不放行**。于是机器人在入口外绕圈直到超时。

这不是 bug，是 constant-speed 模型加 loiter 终端集合的固有保守性。**能刹车的 Dubins4D（QBot）没有这个问题**：它的后路可以是"直接停下"，不需要一块能绕圈的空地。

同理，`unknown="occupied"` 要求**第一帧扫出的自由空间就能放下 loiter circle**，于是 lidar 量程有下限：

```
range ≳ footprint_radius + 2 × 转弯半径 + 余量 ≈ 0.34 m（Burger）
```

量程 0.3 m 时 26 次实验全部 `init_failed`（机器人不出发，这是安全的失败方式）。

## 5.7 失败状态

| 状态 | 触发 | 含义 |
| --- | --- | --- |
| `init_failed` | 第一个周期 `commit_initial` 找不到 witness | 起点无法认证，机器人不出发 |
| `filter_error` | 后续周期 filter 抛 `RuntimeError` | 例如机器人跑到 region 外面 |
| `unsupported` | `NotImplementedError` | filter 不支持的情况（例如非 dubins3d 的 dynamics） |

这三种都**不会悄悄降级**，而是让 episode 停下来并记录原因，避免把"没检查"当成"安全"。
