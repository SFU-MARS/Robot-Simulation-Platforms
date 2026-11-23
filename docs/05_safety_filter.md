# 05 Safety filter：backup filter、witness 与 2D primitive

代码：[`nav_core/safety/backup_filter.py`](../nav_core/nav_core/safety/backup_filter.py)（`BackupFilter`、`Scene`、从地图到面）+ [`nav_core/safety/witness.py`](../nav_core/nav_core/safety/witness.py)（后路搜索）。primitive 的 value 来自离线导出的 `V₀`，由 fast-brt 的 [`fast_brt/query.py`](../fast-brt/fast_brt/query.py) 查询（纯 numpy）。

> 2026-11-22 从 Hu-Hanyang/FaST-BRT 迁过来（roadmap 第 4 步），原名 witness gatekeeper / `FastBrtGatekeeper`。旧仓库不再使用。

## 5.1 安全是怎么定义的

机器人恒速前进、**停不下来**。所以"安全"不能定义成"来得及刹车"，只能定义成：

> **永远存在一条可以一直走下去而不碰障碍的路。**

最简单的"一直走下去"是**绕最小圆**（loiter circle）：方向打死，绕半径 R 的圆一直转。于是得到 backup filter 的规则：

> 接受 nominal 的命令，当且仅当执行完之后，能找到一条 **witness**：
> 一段 rollout 轨迹，全程保持 clearance，**末端停在一个完全自由的 loiter circle 上**。
>
> 找不到就拒绝，改为执行上一周期已经验证过的 witness。

这保证了 **recursive feasibility**：只要出发时有一条 witness，之后每一步要么接受新命令（同时有了新 witness），要么退回旧 witness。旧 witness 是在旧地图上验证的，而地图只会越来越多自由空间（静态环境），所以旧 witness 始终有效。

**安全不依赖 HJ value 算得准不准。** value 只用来提供候选控制、加速搜索；最终放行与否由 witness 的几何验证决定。

## 5.2 一个周期里 backup filter 做什么

`BackupFilter.__call__(x, w_nom, grid)`：

```
1. scene(x, grid)              构造这一帧的 Scene（region + primitive）
2. 首次调用：在起点先找一条 witness；找不到 → RuntimeError（机器人不出发）
3. 模拟 nominal 的这 0.2 秒    arc_step 走 n_sub 个 sub-step，记录最低 margin
4. 从终点找新 witness
      找到 → 放行 w_nom，存下新 witness
      没找到 → 拒绝，返回上一周期存的 witness 的前 n_sub 步（之后接着绕 loiter circle）
5. 返回 n_sub 个 turn rate（物理单位）
```

`n_sub = ctrl_dt / sub_dt`，Burger 是 `0.2 / 0.05 = 4`。所以每个控制周期输出 4 个 zero-order-hold 的 turn rate。

### witness 搜索（`witness.find_witness`）

从给定状态出发，按优先级把一批候选控制序列**一次性批量 rollout**（`arc_step` 是精确的圆弧积分）：

1. **HJ 控制**：最近的那个 primitive 的 `ω = w_max · sign(∂V/∂θ)`（`Scene.hj_control`，闭环，每步重新查询）；
2. 恒定左转 `+1`、恒定右转 `−1`、直行 `0`；
3. "先直行 k 步再打死"的组合（k = 5, 10, 15, …）。

Burger 的配置下 horizon = 2.0 模型时间、`sub_dt = 0.05`，所以 K = 40 步，一共 **17 条几何候选 + 1 条 HJ 候选**。

对每条 rollout：

- 全程 `margin > clearance`（margin 来自 scene，见 §5.4）；
- 终点状态的 **loiter circle**（左右各试一次，72 个采样点）整圈都 `margin > clearance`。

第一条同时满足的就是 witness。都不满足则返回 `None`。clearance 必须盖住采样间隙（`witness_slack`：圆弧上 `dt/2`，圆上 `2R·sin(π/144)`），否则直接报错。

旧仓库里的 B&B 搜索、prefix witness、deadline 等选项没有迁过来（实验里一直用的是 dive）。

## 5.3 2D primitive：一条边一个，value 从同一个 V₀ 查出来

### 为什么需要它

直接对整个场景解一次 HJ value function 是很贵的，而且地图每个周期都在变。做法是：

> 把障碍拆成一条条**面（face）**，每个面对应一个 primitive；
> 所有 primitive 的 value **由同一个离线 `V₀` 平移旋转得到，不用重新求解**。

### 标准 primitive 和长度平移

长度为 L 的 primitive 是一个沿局部 x 轴、很薄的矩形 `|p_x| ≤ L/2, |p_y| ≤ L/(2√α)`，**线性编码**：

```
ℓ_L(p) = max(|p_x|, √α |p_y|) − L/2
```

L 只以常数 `L/2` 出现，所以 `V_L = V₀ − L/2`（`V₀` 是 L = 0 的核），对任意 L 精确成立；位姿平移 `V(x) = V₀(g⁻¹x) − L/2`，`g = (c, β)`。二次编码（`max(p_x², α p_y²) − L²/4`）保留成选项（导出时 `--encoding quadratic`，filter 用 `v0=` 指定文件），用来比较两种编码的差距；它的数值误差大，见 [10 §10.3](10_primitive_alpha_accuracy.md)。

### 离线 V₀

**精度规则**：要进 local update 的 primitive 必须用 `accuracy="low"`（local update 的 kernel 是一阶格式，两边格式不一致时修正区域会扩张到整张网格）。`medium` 的 `V₀` 更准（线性编码前沿平均误差 0.036 R，low 是 0.088 R），只用于点查询，暂时不能用于 local update。backup filter 的安全不依赖 `V₀` 的精度，两种都能用。

`V₀` 在 mars-14 上用 odp 解一次（fast-brt：`python -m fast_brt export-v0 --config configs/v0_dubins_3d.json`），只存最后一层：线性编码、`accuracy="low"`（默认）、241×241×60 网格、[−6, 6]²、α = 20、BRT、T = 2。文件约 14 MB，路径写在 `robots/burger.yaml` 的 `base_value`（`fast-brt/results/v0/…npz`，不进 git）。α 由 `V₀` 决定。在线只做插值，40 个面 × 2000 个状态约 7 ms。

### 内移放置（默认）

对面 `(a, b, n)`（n 指向自由空间），矩形朝向自由空间的那条边和面重合，厚度全在障碍内：`c = (a+b)/2 − n·L/(2√α)`。BRT 朝自由空间一侧的范围只由 L 决定，长面不用切段（[10 §10.2](10_primitive_alpha_accuracy.md)）。`fast_brt.query.PrimitiveV0.place` 还做两件事：

- **短面延长**：短于 `l_min = 2·dx·√α`（0.45 R，Burger 上 4.5 cm）的面对称延长到 `l_min`；
- **长面厚度封顶**：长于 `l_ref`（V₀ 网格可靠区能放下的最长面，4 R = 0.4 m）的面长度不变，厚度封顶为 `l_ref/√α`，面中段取长度 `l_ref` 的面中心处的值。对封顶后的矩形这是精确的，实测 L = 8、16 R 的前沿误差和 L = 4 R 相同（[11](11_roadmap.md) §11.3）。二次编码用 `ℓ_q = ℓ_l(ℓ_l + L)`（严格递增）从参考面换算，连续意义下同样精确；但二次编码的数值误差随 L 变（面越长越准），换算只能沿用 L = 4 R 参考面的误差，所以长面会比直接求解多低估约 0.04–0.06 R（81 网格一阶格式的测试）。线性编码没有这个问题。

离网格边缘 vT 以内的值会被边界条件扭曲，落在这里的点返回下界 `min_{可达范围} ℓ`（保守，且 ≥ 0）。

旧的放法（`placement="centered"`，以面为中线，配合 `cut=True` 把面切成 ≤ `2 h_max √α` 的小段）仍然保留成选项，用来对比。

## 5.4 Scene：把 region 和 primitive 粘起来

累积地图里自由区域可能带洞（孤立的柱子），所以 `Scene` 同时带着 region 的 ring 和 primitive：

| 接口 | 来源 | 作用 |
| --- | --- | --- |
| `margin(xy)` | **region**：到 outer/inner ring 的 signed distance（区域内为正）；再和 primitive 矩形取 min | witness 的碰撞判据，**决定安全** |
| `value(states)` | **primitive**：`V₀` 查询，各面取 min | HJ value（`hj` / `hj_backup` mode 用） |
| `hj_control(states)` | 最近的 primitive 的 `∂V/∂θ` 符号 | HJ 候选 |
| `frames` | primitive 矩形 (center, half_len, half_wid, β) | 画图、检查 |
| `clearance`, `T` | profile 的 1.5 cell；horizon | 采样余量；rollout 长度 |

**这就是 Design decision 1 在代码里的落点**：安全走 region，primitive 只管效率。

## 5.5 关键参数

```python
BackupFilter(profile,
             v0=None,               # PrimitiveV0 或 V₀ 文件；None：profile 的 base_value
             unknown="occupied",    # occupied | free
             primitives="faces",    # faces | rings
             placement="inward",    # inward | centered
             cut=False,             # 只配合 centered 用
             horizon=2.0,           # 模型时间单位
             sub_dt=0.05,
             clearance=None,        # 默认 1.5 cell
             h_max=None,            # cut 时每段伸出的上限，默认 1 cell
             use_hj=True)           # HJ 候选放在最前面
```

| 参数 | Burger 实际值 | 说明 |
| --- | --- | --- |
| `clearance` | 1.5 cell = 3 cm = 0.3 模型单位 | 覆盖栅格分辨率带来的误差：marching squares 的等值线和格心距离可能比真实自由空间多出约 1 个 cell |
| `horizon` | 2.0 模型单位 = 2.0 s | witness 的 rollout 长度，K = 40 步 |
| loiter 半径 R | 1 模型单位 = 0.1 m | 等于转弯半径 |

`primitives` 的两个取值：

- **`"faces"`（默认）**：primitive 来自障碍层的 face（已按 footprint 外推），`unknown="occupied"` 时再加上 frontier 边。同时 `filter.obstacles` / `filter.events` 带稳定 ID，给 leaking-corner 用。
- **`"rings"`**：primitive 直接取 region 每条 ring 的边（早期做法，保留做对比）。

## 5.6 结构性限制：窄通道进不去

loiter circle 需要的自由空间：

```
   圆直径              2R = 0.2 m
 + 车身两侧            2 × 0.105 m
 + clearance 余量      ≈ 0.04 m
 ≈ 0.45 m 宽
```

`hj_reach_target` 里 obstacle 1 和北墙之间只有 **0.4 m**。机器人车身只有 0.21 m 宽，物理上过得去，但进去之后放不下 loiter circle，后路断了，**backup filter 永远不放行**。于是机器人在入口外绕圈直到超时。

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
