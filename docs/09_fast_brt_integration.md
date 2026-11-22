# 09 接入 SFU-MARS/fast-brt：现状、差距与方案（讨论稿）

> 日期：2026-11-21。fast-brt 版本：`main@11ebc27`（"new structure"），已作为 submodule 加在 `fast-brt/`。
> 这是一份**讨论稿**，用来和 fast-brt 的作者对齐接口。下面关于 fast-brt 的描述都来自代码本身，计算耗时还没有实测（laptop 上没有 odp 环境）。

## 9.1 一句话结论

fast-brt 和我们现在用的 FaST-BRT（`Hu-Hanyang/FaST-BRT`）**不是同一套代码，包名都叫 `fast_brt`，接口完全不同**。它的价值在 **leaking corner 的 local update** 和 **Dubins 4D 支持**，这两样我们都没有。

接入的关键观察：**我们的安全保证不依赖 HJ value 的数值**（见 [05](05_safety_filter.md)：安全来自 region 的几何 + witness 验证），所以 fast-brt 的结果**可以慢、可以异步、可以偶尔过时**，只影响效率（介入次数、超时），不影响安全。因此最合适的架构是：**fast-brt 跑在一个独立进程里，按事件异步更新 value，在线 filter 读最新一份结果。**

要做到这一点，fast-brt 需要补几样东西，主要是：**轻量 import、任意长度、任意角度、随机器人移动的局部窗口、增量更新、点查询接口、服务模式**。下面逐条说明。

## 9.2 fast-brt 现在是什么

### 做的事

给一组障碍（每个是 library 里的 primitive + 位姿），算出它们**并集的 value function**，并在 avoid 模式下用 HJ_local_update **修正 leaking corner**：

```python
brt = FastBRT(Config.from_json("configs/dubins_3d.json"))     # 加载 primitive library
result = brt.compute([{"primitive": "line_1m", "center": [0.5, 0.0], "angle": 0.0}, ...])
result.values      # 整张 grid 上的 value，时间在最后一维，index 0 = t=0
```

`compute()`（[`api.py:85`](../fast-brt/fast_brt/api.py#L85)）四步：

1. **place**：每个障碍的 center **snap 到整格**（[`api.py:79`](../fast-brt/fast_brt/api.py#L79)），角度是 π/2 的整数倍时精确旋转，否则对整个数组插值（[`transforms.py:108`](../fast-brt/fast_brt/core/transforms.py#L108)）；
2. **union**：取 min；
3. **local update**（avoid 且 ≥ 2 个障碍）：`get_leaking_set` 找"最小值和次小值相遇"的点，HeteroCL kernel 在那里修正。中间结果写一个临时 `.npy` 文件交给 HJ_local_update（[`local_update.py:42`](../fast-brt/fast_brt/core/local_update.py#L42)）；
4. **crop** 回 grid。

### 关键约束（都来自代码）

| 约束                                                                                                                | 位置                                                                                                 | 影响                                                                                          |
| ------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| `import fast_brt` 会立刻 import odp 和 HJ_local_update（HeteroCL）                                                | [`config.py:12`](../fast-brt/fast_brt/config.py#L12)、`core/hj_local_update.py`                   | **只能在 Python 3.8 的 odp 环境里运行**，ROS 2 节点（Python 3.10 / 3.12）里 import 不了 |
| primitive 是**有名字、固定长度**的（如 `line_1m`）                                                          | `configs/primitives.json`                                                                          | 不同长度要分别预计算                                                                          |
| 角度只有 π/2 的整数倍是精确的；其它角度插值，3D 约 0.2 s/个、4D 约 2 s/个，误差最大 0.16–0.26                     | README "Known issues"                                                                                | 我们的 face 是任意角度                                                                        |
| center snap 到整格                                                                                                  | [`api.py:79`](../fast-brt/fast_brt/api.py#L79)                                                      | 位置误差 ≤ 半格                                                                              |
| grid 固定、以原点为中心；障碍必须在 grid 内，**碰到 grid 边缘直接报错**                                       | [`api.py:109`](../fast-brt/fast_brt/api.py#L109)、[`api.py:114`](../fast-brt/fast_brt/api.py#L114) | 地图比 grid 大时没法用                                                                        |
| 每次`compute()` 从头算，没有障碍 ID、没有缓存                                                                     | [`api.py:85`](../fast-brt/fast_brt/api.py#L85)                                                      | 每周期全量重算                                                                                |
| 输出是整张 grid 数组                                                                                                | `Result`                                                                                           | 我们需要的是任意状态点的值和梯度                                                              |
| HJ_local_update 不是 package，通过`HJ_LOCAL_UPDATE_PATH`（默认 `~/Documents/HJ_local_update`）加进 `sys.path` | `core/hj_local_update.py`                                                                          | 部署时路径要手动配                                                                            |
| x、y 点数必须相同且为奇数，θ 点数是 4 的倍数                                                                       | [`config.py:199`](../fast-brt/fast_brt/config.py#L199)                                              | 配置限制                                                                                      |

### 好消息

- **单位已经一致**：示例 config 是 `v = 1, wMax = 1`，和我们的无量纲约定相同（转弯半径 = 1）；
- **primitive 的数学形式一致**：都是 `max(α_x x², α_y y²) − L²/4`（quadratic encoding，[`primitive.py:35-38`](../fast-brt/fast_brt/core/primitive.py#L35-L38)）；
- **有 Dubins 4D**（`x, y, v, θ`，带 `aMax / vMin / vMax`），正是 QBot 要的；
- 结构清晰，测试齐全，README 写得很清楚。

## 9.3 我们在线导航需要什么

每个控制周期（Burger 是 0.2 s）：

| 我们有的输入                                                                   | 来源                                  |
| ------------------------------------------------------------------------------ | ------------------------------------- |
| 障碍的**face**：任意端点 `(a, b)`、任意长度和角度，已按 footprint 外推 | `extract_obstacles(grid, offset=r)` |
| 每个 face 的**稳定障碍 ID**、本周期的**变化事件**                  | `ObstacleTracker`                   |
| **corner** 列表（convex / concave）                                      | 障碍层                                |
| 机器人位姿、地图范围                                                           |                                       |

| filter 需要的输出                          | 用途                                             |
| ------------------------------------------ | ------------------------------------------------ |
| `U(x)`：任意状态的组合 value             | B&B 剪枝（`search="bnb_U"`）、单调性诊断       |
| `∂U/∂θ` 的符号（或绑定 primitive 的） | `hj_switch` 候选控制（witness 搜索的第一候选） |
| `U_s(x)`：剩余时域 s 时的 value          | B&B                                              |

查询量：每次 witness 搜索约 18 条候选 × 40 步，加上 loiter circle 采样，是**几千个点的批量查询**，不是整张 grid。

规模：地图 2.1 m（= 21 模型单位）到 4.2 m（42 单位），以后更大。但 BRT horizon = 2 模型时间，机器人在这段时间里最多移动约 2 个单位，**只有机器人周围几个单位以内的障碍才影响当前决策**。

时间预算：整个决策要在 200 ms 内完成（现在 15–35 ms）。

## 9.4 关键观察：fast-brt 可以异步

我们的 gatekeeper（[05 §5.4](05_safety_filter.md)）把两件事分开了：

|                      | 依赖                                                 | 决定什么                                       |
| -------------------- | ---------------------------------------------------- | ---------------------------------------------- |
| `margin(xy)`       | **region 的几何**（detected 格子以外全算障碍） | **安全**：witness 是否成立               |
| `U`、`hj_switch` | HJ value                                             | **效率**：先试哪条候选、B&B 剪掉哪些分支 |

所以 HJ value **算错、算慢、过时，都不会导致碰撞**，只会让 filter 多试几条候选或多介入几次。这意味着：

- fast-brt 不必在 200 ms 内返回，可以按**事件触发**、低频更新（例如只在障碍 `new` / `updated` 时重算）；
- 在线 filter 永远用**最新一份可用的结果**，新结果没到时用旧的，或退回到纯 transport（现在的做法）；
- fast-brt 可以放在**另一个进程、另一个 Python 环境**里，完全不用进 ROS 节点。

local update 能带来的好处（需要实验验证）：min-composition 在 corner 处会 leak（把注定要撞的状态算成安全），导致 `hj_switch` 在 corner 附近给出错误的逃离方向、B&B 剪枝变弱。修正后，**候选质量更高、witness 搜索更快、介入和超时更少**。

## 9.5 差距清单

按对在线使用的影响排序。"在哪做"一列是建议：大部分改在 fast-brt 里更合适，因为约定（时间轴顺序、θ 周期、α 方向）归它管。

| #   | 差距                                                                                                                                                                                            | 我们需要                                               | 建议                                                                                                                                                                        | 在哪做                     | 工作量 |
| --- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------- | ------ |
| G1  | **任何** `fast_brt.*` 模块都离不开 HeteroCL：`fast_brt/__init__.py` 会立刻 import `api → config → core/hj_local_update`，连 `fast_brt.core.transforms` 都 import 不了（已实测） | 至少"加载 library + 放置 + 并集 + 查询"不依赖 HeteroCL | 保留她"HJ_local_update 只在`core/hj_local_update.py` 一处 import"的设计，只把这一处改成**首次调用时才 import**；其余文件不动                                        | fast-brt                   | 小     |
| G2  | 只有固定长度的命名 primitive                                                                                                                                                                    | 任意长度 L                                             | quadratic encoding 下长度只是常数平移：**解一次 L=0 的核 `max(α_x x², α_y y²)`，任意 L 都是 `V − L²/4`**（精确，无插值）。obstacle 输入加 `"length"` 字段 | fast-brt                   | 小     |
| G3  | 非 π/2 角度慢且不准                                                                                                                                                                            | 任意角度                                               | (a) precompute 时按 K 个角度各解一份（README 已提到），放置时取最近角度，只剩平移；(b) 点查询走 transport（见 G6），只在 local update 区域内才需要整张数组                  | fast-brt                   | 中     |
| G4  | grid 固定、障碍碰边报错                                                                                                                                                                         | 跟着机器人走的**局部窗口**                       | `compute(obstacles, origin=robot_xy)`：先把障碍平移到窗口坐标再 snap；丢掉影响不到窗口的障碍；碰到窗口边缘的障碍允许保留（边缘本来就靠 margin 处理）                      | fast-brt                   | 中     |
| G5  | 每次全量重算                                                                                                                                                                                    | 增量更新                                               | 有状态的 scene：按障碍 ID 缓存 placed 数组和并集；只有变化的障碍重新 place；leaking set 没变就跳过 local update                                                             | fast-brt                   | 中     |
| G6  | 只输出整张数组                                                                                                                                                                                  | 任意状态的`V`、`∂V/∂θ`、指定时间片              | `Result.value_at(states, s=None)`、`Result.dtheta_at(states)`：三线性插值，处理 θ 周期和时间轴                                                                         | fast-brt（或我们这边）     | 小     |
| G7  | 只能同进程调用                                                                                                                                                                                  | ROS 节点（另一个 Python）能用                          | `python -m fast_brt serve`：常驻进程，加载一次 library，接收 `{obstacles, origin}` 请求，返回数组（共享内存或 socket）                                                  | fast-brt                   | 中     |
| G8  | 耗时未知                                                                                                                                                                                        | 知道在我们的规模下要多久                               | 在 odp 机器上测：窗口 ±4 单位、61–121 点/轴、θ 32–60、lookback 2.0、10–50 个障碍                                                                                       | 实测                       | 小     |
| G9  | HJ_local_update 靠`~/Documents` 路径                                                                                                                                                          | 可复现部署                                             | 作为 fast-brt 的 submodule，或做成可 pip 安装的 package，pin 版本                                                                                                           | fast-brt / HJ_local_update | 小     |
| G10 | 约定没对齐                                                                                                                                                                                      | 一份明确的接口约定                                     | 见 §9.6                                                                                                                                                                    | 双方                       | 小     |
| G11 | 包名冲突                                                                                                                                                                                        | 两份`fast_brt` 不能同时在 path 上                    | 见 §9.7 的选择                                                                                                                                                             | 双方                       | 视方案 |
| G12 | `.claude/settings.local.json` 被提交了（含 `/localhome/cha198/...` 本机路径）                                                                                                               | —                                                     | 加进`.gitignore`                                                                                                                                                          | fast-brt                   | 极小   |
| G13 | odp 版本没有 pin：fast-brt 调用`Grid(min, max, dims, pts, pDim)`，而 optimized_dp 最新版在 `689548e`（"Remove dim from grid initializations"）去掉了 `dims` 参数                          | 可复现的环境                                           | 在 README / pyproject 里写明 odp 的 commit（`689548e` 之前），或改用新签名                                                                                                | fast-brt                   | 极小   |

**G2 值得单独说明**：两边的 primitive 都是 `max(α_x x², α_y y²) − L²/4`，长度只以常数 `L²/4` 出现。HJ 方程对 terminal cost 的常数平移是精确传递的（BRT 取的是沿轨迹的最小值，减一个常数后最小值也只减这个常数），所以 **一份核 value 就覆盖所有长度**。`Hu-Hanyang/FaST-BRT` 就是这么做的（"exact length shift"）。唯一要注意的是 padding 要按最大长度留够。

## 9.6 需要对齐的约定（G10）

| 项目               | fast-brt 现状                                                            | 我们现状                                     | 建议                                                                                                              |
| ------------------ | ------------------------------------------------------------------------ | -------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| primitive 长轴方向 | `alpha = [α_x, α_y]`；示例 `[10, 1]` 是**沿 y 长、x 方向薄** | 沿局部 x 长、`α` 作用在 y 上，`α = 20` | 定一个：比如"angle = 0 时线段沿 x 轴"，face`(a, b)` → `center = (a+b)/2`、`angle = atan2(b−a)`、`length = |
| 细长比             | `[10, 1]` → 半宽 = L/(2√10)                                          | `α = 20` → 半宽 = L/(2√20)              | 统一，或作为 library 的参数                                                                                       |
| 单位               | `v = 1, wMax = 1`                                                      | 无量纲，转弯半径 = 1                         | 已一致；我们负责物理单位换算                                                                                      |
| 状态顺序（4D）     | `(x, y, v, θ)`，θ 是第 4 维                                          | profile 写的是`(x, y, θ, v)`              | 以 fast-brt 为准，我们改                                                                                          |
| value 符号         | avoid：V > 0 安全                                                        | 同                                           | 已一致                                                                                                            |
| 时间轴             | 最后一维，index 0 = t = 0                                                | `BaseValue.slices[j]`，j = s / slice_dt    | 查询接口里统一用"剩余时域 s"                                                                                      |
| 坐标原点           | grid 中心                                                                | 世界坐标                                     | 由`origin` 参数换算（G4）                                                                                       |

## 9.7 包名冲突：两种选择（G11）

`Hu-Hanyang/FaST-BRT` 里有我们在线 filter 依赖的全部代码（`BaseValue`、`WitnessGatekeeper`、`SceneModel`、`certify`），而 fast-brt 里没有这些。两个包都叫 `fast_brt`，同时在 `sys.path` 上时谁在前用谁，另一个就坏了。

**方案 A（推荐）：合进 SFU-MARS/fast-brt，分两层**

```
fast_brt/
  core/, api.py, library.py     # 现有：离线 precompute + local update（需要 odp / HeteroCL）
  online/                       # 新增：纯 numpy，不 import odp
    transport.py                #   核 value 的点查询：V(g⁻¹x) − L²/4（来自 Hu-Hanyang/FaST-BRT 的 BaseValue）
    query.py                    #   Result 的插值查询（G6）
    scene.py                    #   RegionScene（目前在本 repo 的 nav_core/safety）
    gatekeeper.py, witness.py   #   witness gatekeeper（来自 Hu-Hanyang/FaST-BRT）
  service.py                    # 服务模式（G7）
```

好处：一个包、一份约定、一处维护。Robot-Sim 只依赖 SFU-MARS/fast-brt 一个 submodule。在线进程只 import `fast_brt.online`，不碰 HeteroCL。

代价：要把 Hu-Hanyang/FaST-BRT 的在线部分搬过来（规模不大：`primitive.py` 的 transport、`filter/gatekeeper.py`、`filter/scene_model.py`、`certify/witness.py` 和它们的依赖）。

**方案 B：两个包并存，改名其一**

例如把 Hu-Hanyang/FaST-BRT 的包改名为 `fast_brt_online`，两个都作为 submodule。改动小，但两套约定要分别维护，长期容易漂移。

## 9.8 建议的整体架构

```
┌──────────────── ROS 2 节点 / nav_core（Python 3.10+，纯 numpy）────────────────┐
│                                                                                 │
│  /scan → OccupancyGrid → extract_obstacles → ObstacleTracker ──events──┐        │
│                              │                                         │        │
│                              ▼                                         │        │
│                        region（几何）                                   │        │
│                              │                                         │        │
│  nominal (MPPI) → w_nom → gatekeeper ── margin：region 几何（安全）      │        │
│                              │      └── U / hj_switch：HJProvider       │        │
│                              ▼               ▲                         │        │
│                           /cmd_vel           │ 最新一份 value          │        │
└──────────────────────────────────────────────┼─────────────────────────┼────────┘
                                               │                         │
                                     共享内存 / socket（异步）            │
                                               │                         ▼
┌──────────────── fast-brt serve（odp 环境，Python 3.8 + HeteroCL）─────────────────┐
│  按 ID 缓存 placed primitive → union → local update（leaking corner）→ value 窗口  │
└───────────────────────────────────────────────────────────────────────────────────┘
```

- 事件触发：只有障碍 `new` / `updated` / `merged` 时才发请求；
- 在线 filter 用 `HJProvider` 取 value：fast-brt 的结果到了就用它，没到就用纯 transport（现在的做法），两者都不影响安全；
- 4D（QBot）时，fast-brt 提供 4D value，我们这边要补 4D 的 witness（终端集合可以是"停下来"，不再需要 loiter circle）。

## 9.9 我们这边（Robot-Sim）要做的 refactor

| #  | 内容                                     | 说明                                                                                                                                                                                                                                      |
| -- | ---------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| R1 | **`HJProvider` 接口**            | `RegionScene` 现在直接用 `BaseValue` 算 `U` / `hj_switch`。抽出一个接口 `values(states, s)`、`dtheta(states)`，两个实现：`TransportProvider`（现状）和 `FastBrtProvider`（读 fast-brt 结果）。gatekeeper 和 margin 不用改 |
| R2 | **face → fast-brt obstacle 转换** | `(a, b)` → `{length, center, angle}`，按 §9.6 的约定，换到窗口坐标，过滤掉影响不到窗口的 face                                                                                                                                       |
| R3 | **异步客户端**                     | 订阅`ObstacleTracker` 的事件，向 fast-brt 服务发请求，后台接收结果；带超时和"结果新鲜度"标记                                                                                                                                            |
| R4 | **对比实验**                       | 同一组场景下比较 transport-only 和 transport + local update：介入率、超时、witness 搜索的 rollout 数、B&B 节点数。这是证明 local update 价值的实验                                                                                        |

R1 不依赖 fast-brt 的任何改动，可以先做。

## 9.10 分阶段计划

| 阶段         | 内容                                                                                                                                                                            | 依赖 |
| ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---- |
| **P0** | 在 odp 机器上实测`compute()` 在我们规模下的耗时（G8）；对齐约定（G10）；决定包的组织方式（G11）                                                                               | 讨论 |
| **P1** | fast-brt：轻量 import（G1）、任意长度（G2）、点查询（G6）、`.gitignore`（G12）、odp 版本（G13），然后 `origin` 窗口（G4）。Robot-Sim：`HJProvider`（R1）、face 转换（R2） | P0   |
| **P2** | fast-brt：服务模式（G7）、增量缓存（G5）。Robot-Sim：异步客户端（R3）、对比实验（R4）                                                                                           | P1   |
| **P3** | 任意角度（G3）；QBot 4D：profile、MPPI 的 4D step、4D witness                                                                                                                   | P2   |

## 9.11 复用优先的实现方式

原则：**在她的接口上扩展，不另起一套**。她的文件尽量不动，必须改的地方只做最小改动；新功能放在新函数或新文件里，直接调用她已有的函数。

| 差距           | 新增 / 改动                                                                                                         | 复用她的代码                                                                                   |
| -------------- | ------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| G1 轻量 import | 只改`core/hj_local_update.py`：把顶层 import 包进一个首次调用才执行的函数                                         | 保留"HJ_local_update 只在一处 import"的设计；其它文件一行不改                                  |
| G2 任意长度    | `compute()` 的 obstacle 接受可选的 `"length"`；放置后对 `values` 和 `initial` 加常数 `(L_lib² − L²)/4` | `place()`、library、`primitive_2d` 的 quadratic 形式原样使用                               |
| G4 局部窗口    | `compute(obstacles, origin=...)`：snap 之前先减去 `origin`；`Result` 记下 `origin`                          | `snap()`、`place()`、`crop()` 原样使用                                                   |
| G5 增量缓存    | 把`compute()` 里"放置一个障碍"那段抽成方法，按 `(primitive, cells, angle, length)` 缓存                         | `place()`；local update 照旧调用                                                             |
| G6 点查询      | 新文件`fast_brt/query.py`：`value_at(result, states, t=-1)`、`dtheta_at(...)`                                 | 照搬`rotate()` 处理 θ 周期的做法（在末尾补一份第一个 θ 切片再用 `map_coordinates` 插值） |
| G3 任意角度    | `primitive_2d` 加一个 `angle` 参数，`precompute` 按 K 个角度各解一份                                          | `precompute()`、`solve_hj`、library 格式                                                   |
| G7 服务模式    | 新文件`fast_brt/service.py`，`cli.py` 加一个 `serve` 子命令                                                   | `FastBRT`、`Config`、现有 CLI 结构                                                         |

**可以在没有 HeteroCL 的机器上测试的部分**：G1 改完之后，G2 / G4 / G5 / G6 都可以用"拿 `primitive_2d` 现场构造的假 library"做单元测试，不需要真正求解 HJ。需要 HJ 求解和 local update 的测试（她现有的 `-m slow` 测试）仍在 odp 机器上跑。

**建议第一批**（改动小、不需要先做设计决定）：G1、G2、G6、G12、G13。G4 要先和她商量"碰到窗口边缘的障碍怎么处理"（她现在是直接报错，因为边缘附近的值不可靠）。

## 9.12 讨论时要问的问题

1. **定位**：fast-brt 是离线工具，还是要在线跑？她预期一次 `compute()` 在什么规模下多少秒？
2. **local update 的耗时构成**：HeteroCL kernel 是首次编译后缓存的吧？每次调用的 kernel load + correction 大概多久？临时文件那一步能不能换成直接传数组？
3. **任意长度**：同意用 `V − L²/4` 的长度平移吗？padding 按最大长度留够有没有问题？
4. **任意角度**：倾向"预计算多个角度"还是"点查询 transport"？local update 的区域内要不要整张数组？
5. **局部窗口**：grid 能不能随机器人移动（`origin` 参数），碰到窗口边缘的障碍能不能保留？
6. **增量**：leaking set 在障碍没变时能不能复用，local update 能不能只在变化的 corner 附近重跑？
7. **约定**：primitive 长轴方向、α 取值、4D 状态顺序，按 §9.6 哪一种？
8. **包的组织**：方案 A（合进 fast-brt，分 online / offline 两层）还是方案 B（两包并存、改名）？
9. **HJ_local_update 和 odp 的版本**：HJ_local_update 能不能作为 fast-brt 的 submodule 或做成 package？她用的 odp 是哪个 commit（`Grid` 还带 `dims` 参数的版本）？
10. **4D**：QBot 的 dynamics 参数（`aMax`、`vMin`、`vMax`、`wMax`）定了吗？
