# 11 路线图：已完成、进行中、待办

> 更新：2026-11-22。这一篇是整个项目的进度总表，细节在各篇文档里。

## 11.1 目标

对 Dubins3D、Dubins4D，以及未来的 6D 无人机：在**未知环境**里用 nominal controller 导航，边走边建图，用 **2D primitive** 的 BRT 保证安全；primitive 组合在拐角处 leak 时，用 **online local update** 修正。

论文主线：**2D primitive（离线一次）+ 在线 local update 修 leaking corner**。filter 有三种做法，都保留为选项。

## 11.2 已定的决策

| 决策             | 内容                                                                                                                                                                   | 出处                                       |
| ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------ |
| repo 分工        | primitive / value function 相关的放**fast-brt**（我们自己的 `robot-sim` 分支），感知、建图、filter 和 ROS 放 **Robot-Sim**；`Hu-Hanyang/FaST-BRT` 退役 | [09](09_fast_brt_integration.md)            |
| primitive 放置   | **内移**：矩形朝向自由空间的边和检测到的面重合，BRT 只由长度 L 决定，长面不切段                                                                                  | [10 §10.2](10_primitive_alpha_accuracy.md) |
| 离线`V₀`      | **线性编码 + odp `accuracy="low"` + 241 网格 + α=20 + BRT**（2026-11-22 从 medium 改成 low：和 local update 的一阶格式一致，全程只用一份 `V₀`）；线性 / 二次 × low / medium 四份都保留成选项，用来比较差距 | [10 §10.3](10_primitive_alpha_accuracy.md) |
| 精度和 local update | 要进 local update 的 primitive **只能用 `low`**（kernel 是一阶格式）；`medium` 的 `V₀` 只用于点查询，暂时不用于 local update；要用二阶得改 kernel | [05 §5.3](05_safety_filter.md) |
| 短面             | 短于`L_min ≈ 2·dx·√α`（约 0.45 R）的面，对称延长到 `L_min`                                                                                                    | [10 §10.1](10_primitive_alpha_accuracy.md) |
| primitive 用 BRT | BRS 在面前方的高度为 0，不能用于避障                                                                                                                                   | [10 §10.3](10_primitive_alpha_accuracy.md) |
| filter           | 三种都保留为选项：`hj`（经典 HJ filter）、`backup`（后路验证，原 witness gatekeeper）、`hj_backup`（**HJ 为主、backup 兜底**：HJ 给出这一周期的候选控制，backup 检查走完后有没有 witness，2026-11-22 决定） | [05 §5.8](05_safety_filter.md)                   |
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
- **fast-brt（`robot-sim` 分支，`0d305d6`）：第 1–3 步**
  - `python -m fast_brt export-v0 --config configs/v0_dubins_3d.json [--alpha 20 --encoding linear --accuracy low]`（默认值）：用 odp `HJSolver` 求 L=0 的核，只存最后一层（`results/v0/*.npz`，含网格参数和 meta）。推荐配置 `configs/v0_dubins_3d.json`：241×241×60、[−6, 6]²、T=2、BRT；
  - `fast_brt.query.PrimitiveV0`（纯 numpy，不 import odp / HeteroCL / scipy）：`place(a, b, n)` 做内移放置和短面延长（`l_min = 2·dx·√α`），`value` / `values` / `query` 给出 `V`、最近的面、`∂V/∂θ`（4D 还有 `∂V/∂v`）和 bang-bang 的 `u*`；`Faces.rectangles()` 给出和 `primitive_frames` 同格式的矩形；`fast_brt` 的 `__init__` 改成按需 import，`import fast_brt.query` 不再拉起 odp；
  - **长面**：长于 `l_ref`（V₀ 网格可靠区能放下的最长面，推荐配置下 4 R）的面，长度不变、**厚度封顶**为 `l_ref/√α`，面中段取长度 `l_ref` 的面中心处的值。对封顶后的矩形，这在连续意义下是精确的（要求 `l_ref ≥ 2vT`）；内移放置下厚度在面后方，只影响"薄墙背面穿出"。二次编码通过 `ℓ_q = ℓ_l(ℓ_l + L)` 换算，也支持长面，但沿用参考面的数值误差（长面比直接求解多低估约 0.04–0.06 R，81 网格一阶格式的测试；线性编码没有这个问题）；
  - **网格边缘**：离边缘 `vT` 以内的值会被边界条件扭曲，落在这里的点返回下界 `min_{可达范围} ℓ`（保守，而且对 `l_ref` 以内的面总是 ≥ 0，远处不会被误判为不安全）；
  - 本地验证（旧 FaST-BRT 的 numpy 一阶求解器代替 HJSolver，81 网格，T=1）：查询值和"在世界网格上直接求解放好的矩形"相比，面附近 `|ΔV| ≤ 0.14`（来自插值，矩形中心不在格点上），符号不一致 ≤ 0.03%（只出现在长面中段，幅度 < 0.09），`∂V/∂θ` 的符号和 `u*` 一致率 ≥ 99.9%；40 个面 × 2000 个状态的 `query` 约 7 ms；
  - 测试：`tests/test_query.py`、`tests/test_v0.py`（快速）；`pytest -m slow -s -k direct_solve`（mars-14，用真正的 HJSolver，low 和 medium）。
  - **mars-14（2026-11-22）**：slow 测试两种精度都通过。面附近 `max|ΔV|` 为 low 0.10–0.19、medium 0.11–0.13，符号不一致 low ≤ 0.047%、medium ≤ 0.006%（都在斜放的面上，是格式对旋转不完全不变造成的）；
  - **推荐配置的 `V₀` 已导出**：`fast-brt/results/v0/v0_dubins_3d_linear_alpha20_medium.npz`（13.9 MB，求解 8.0 s，124 步；`results/` 不进 git，笔记本上有一份）。`l_min` = 0.447 R，`l_ref` = 4 R。车头正对面时的 BRT 前沿高度和精确值相比：

    | L（R） | 0.5 | 1 | 2 | 4 | 8 | 16 |
    | --- | --- | --- | --- | --- | --- | --- |
    | 中心误差 | −5.4% | −4.8% | −4.5% | +0.4% | +0.4% | +0.4% |
    | 整条前沿最大误差（R） | 0.040 | 0.041 | 0.055 | 0.056 | 0.056 | 0.056 |

    L ≤ 4 和 [10 §10.3](10_primitive_alpha_accuracy.md) 的表一致，说明导出和查询都没有引入额外误差；L = 8、16 走的是厚度封顶，精度和 L = 4 相同。
  - **四份 `V₀` 都已导出**（线性 / 二次 × low / medium，`fast-brt/results/v0/`，各 13.9 MB，求解 3.6–8.0 s）。车头正对面时前沿的平均误差（R，负值 = BRT 算小了）：

    | 编码 / 精度 | L=0.5 | 1 | 2 | 4 | 8 | L ≤ 4 平均 |
    | --- | --- | --- | --- | --- | --- | --- |
    | **线性 / low（默认）** | −0.062 | −0.085 | −0.114 | −0.086 | −0.049 | −0.087 |
    | 线性 / medium | −0.037 | −0.034 | −0.043 | −0.028 | −0.011 | −0.036 |
    | 二次 / low | −0.119 | −0.143 | −0.174 | −0.143 | −0.129 | −0.145 |
    | 二次 / medium | −0.046 | −0.044 | −0.055 | −0.036 | −0.022 | −0.045 |

    L ≤ 4 的平均值和 [10 §10.3](10_primitive_alpha_accuracy.md) 的表一致。默认用线性 / low：和 local update 的一阶格式一致，代价是前沿平均多低估约 0.05 R（Burger 上 5 mm）。

- **Robot-Sim（`4a60ff5`）：第 4 步**
  - `nav_core/safety/backup_filter.py`：`BackupFilter`（原 `FastBrtGatekeeper`）、`Scene`（原 `RegionScene`）、`place_faces`、`load_v0`；`nav_core/safety/witness.py`：`find_witness`（只迁了实际在用的 dive 搜索，B&B / prefix / deadline 没迁）和精确圆弧积分 `arc_step`；旧的 `fastbrt_gatekeeper.py` 删掉了；
  - HJ 项改用 `fast_brt.query.PrimitiveV0`，`V₀` 从 profile 的 `base_value` 读（`robots/burger.yaml` 指向 `fast-brt/results/v0/v0_dubins_3d_linear_alpha20_low.npz`，文件不进 git）；`fast_brt` 从 PYTHONPATH 或 `fast-brt/` 子模块导入，旧仓库误放在 PYTHONPATH 上时报错；
  - 默认值已经是**内移、不切段**（新 `V₀` 只有内移的放置，`centered` + `cut` 保留成选项）；filter 的名字在 CLI / ROS 参数里从 `gatekeeper` 改成 `backup`，ROS 节点多了 `v0` 参数；
  - 测试：nav_core 47 个全过（无 `V₀` 文件时闭环测试 skip，几何测试用假 `V₀`），不再需要旧仓库；
  - `experiments/alpha_study.py` 依赖旧仓库的 numpy 求解器和旧 adapter，标成 legacy（用 `daac2c4` 的 worktree 复现 doc 10）；新的批量脚本 `experiments/filter_batch.py`（同样的场景和指标，`--v0` 可以传多个文件比较编码 / 精度 / α）；
  - **闭环对比**（MPPI + backup filter，unknown occupied，和 [10 §10.5](10_primitive_alpha_accuracy.md) 同样的起点和目标；新 `V₀` = 线性、medium、241、α=20）：

    | 场景 | 配置 | 到达 / 碰撞 / 超时 | 介入率 | primitive 数 | 最小距离 | ms/step |
    | --- | --- | --- | --- | --- | --- | --- |
    | hj（27 次） | 内移，旧实现 | 16 / 0 / 11 | 0.049 | 12.8 | 4.0 cm | 20.8 |
    | | **内移，迁移后** | 16 / 0 / 11 | 0.049 | 12.8 | 4.0 cm | 19.1 |
    | | 对称 + 切段，迁移后 | 16 / 0 / 11 | 0.072 | 49.3 | 5.5 cm | 20.9 |
    | 柱子（28 次） | 内移，旧实现 | 28 / 0 / 0 | 0.004 | 15.5 | 4.4 cm | 32.0 |
    | | **内移，迁移后** | 28 / 0 / 0 | 0.004 | 15.5 | 4.4 cm | 21.7 |
    | | 对称 + 切段，迁移后 | 28 / 0 / 0 | 0.012 | 49.9 | 5.8 cm | 23.0 |
    | 遮挡（23 次） | 内移，旧实现 | 16 / 0 / 7 | 0.068 | 36.4 | 3.2 cm | 36.2 |
    | | **内移，迁移后** | 16 / 0 / 7 | 0.064 | 36.7 | 3.2 cm | 34.5 |
    | | 对称 + 切段，迁移后 | 16 / 0 / 7 | 0.071 | 96.0 | 5.0 cm | 38.3 |

    迁移后的结果和旧实现基本一致，没有碰撞。和 doc 10 的结论一样：backup filter 的安全靠几何，`V₀` 的变化（二次 / low / 121 → 线性 / medium / 241）对闭环几乎没有影响。

    换成默认的线性 / low `V₀` 后重跑（`experiments/filter_batch.py`）：三个场景的到达 / 碰撞 / 超时、介入率、primitive 数和最小距离都和 medium **完全相同**（hj 16 / 0 / 11、0.049；柱子 28 / 0 / 0、0.004；遮挡 16 / 0 / 7、0.064），ms/step 19.7 / 21.0 / 34.0。

- **Robot-Sim（未提交）：第 5 步**
  - 默认值：内移、不切段、`V₀` = 线性 / low（`robots/burger.yaml`）；
  - 用新默认值重跑了 README 和 doc 04 里的三组批量（`experiments/filter_batch.py`，原来的脚本没进仓库，用例集合换成了 doc 10 的那套），原始数据在 `docs/figures/batch_*.csv`：两种 unknown 模式和两种 nominal 加 backup filter 都没有碰撞；遮挡场景量程 0.3 m 时 `unknown=occupied` 不出发、`unknown=free` 撞 2 次，0.5 / 0.8 m 两种模式都安全；结论和原来一样；
  - doc 10 的结论（推荐配置、默认值、下一步）、doc 04 / 05 的数字、README 的三张表都已更新。

- **Robot-Sim + fast-brt：review 后的修正（2026-11-22）**
  - `hj` 不再构造 region（只用 primitive），离开 region 不再中止 run：以前的"擦边"现在会走到真正的碰撞，统计更干净；
  - 阈值改成**距离余量** `margin`（米）：fast-brt 的 `PrimitiveV0.margin` 把每个面的值换回线性编码再除以 `max(1, √α)`（V 在位置上是 √α-Lipschitz 的，所以余量 d 表示同朝向 d 以内的状态都在 BRT 外），两种编码含义相同；`hj` 默认 0，`hj_backup` 默认 2 cm；
  - `robust=True`：余量再加上 clearance（1.5 格）和子步之间的采样间隙 `(v + v·T·ω_max)·dt/2`（`|∂V/∂θ| ≤ √α·v·T`）；
  - V₀ 路径：`base_value` 相对 profile 所在仓库（`profile.root`）解析，环境变量 `ROBOT_SIM_V0` 可以覆盖；测试 fixture 不再依赖导入顺序（`import_query`）；
  - V₀ 文件改为 **GitHub release** 存放：`robots/v0_release.yaml` 记 tag 和 sha256，`nav_core/scripts/fetch_v0.py` 下载并校验，加载登记过的文件时校验 sha256；CI 设 `NAV_CORE_REQUIRE_V0=1` 时缺 V₀ 直接失败。

## 11.4 进行中（下一步，已同意）

| # | 任务                                                                                                                                                                       | 在哪      | 依赖 |
| - | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------- | ---- |
| 1 | ✅ **导出离线 `V₀`**：新增 CLI，按 config 求解单个 primitive 的 **BRT**（可选 `accuracy`），只保存最后一层和网格参数                                         | fast-brt  | —   |
| 2 | ✅ **在线点查询**（纯 numpy，不 import odp）：长度平移、内移放置、短面延长、`V`、`∂V/∂θ`、`u*`                                                                 | fast-brt  | —   |
| 3 | ✅ **在 mars-14 上导出推荐配置的 `V₀`**：`python -m fast_brt export-v0 --config configs/v0_dubins_3d.json`，先跑 `pytest -m slow -s -k direct_solve`                                                                                                                            | mars-14   | 1    |
| 4 | ✅ **把 backup filter 从 FaST-BRT 迁到 Robot-Sim**（`nav_core/safety`），改名 `BackupFilter`；HJ 项改用第 2 步的点查询。两个包都叫 `fast_brt`，不迁移就没法同时用 | Robot-Sim | 2    |
| 5 | ✅ **切换默认值**（内移、不切段、加载 `V₀` 文件），重跑批量实验，更新文档                                                                                            | Robot-Sim | 3、4 |

## 11.5 待办

**filter 和 local update**

- [X] `hj` 和 `hj_backup` 两种 filter mode（`nav_core/safety/hj_filter.py`，2026-11-22）：`hj_backup` = HJ 为主、backup 兜底，`margin` 必须 > 0（默认 2 cm），见 [05 §5.8](05_safety_filter.md)；批量对比（margin 改成距离、`hj` 不再构造 region 后重跑）：`hj` 在有拐角的场景（hj、遮挡）里有 4–10 次碰撞，从不超时；`hj_backup` 三个场景都没有碰撞，到达数和 `backup` 接近，介入率高（0.13–0.47，HJ 提前避让）。诊断（`experiments/hj_failures.py`）：11 次碰撞里 9 次是 leaking corner，1 次单个面 `V₀` 低估，1 次 `U ≤ 0` 后逃逸太晚；`robust` 反而让失败变成 28 次（27 次拐角）；`hj_backup` 三个场景都没有碰撞，到达数和 `backup` 基本相同，介入率高（0.17–0.48，HJ 提前避让）；
- [X] local update 的 BRT 版本（HJ_local_update `robot-sim` 分支，`brt=True`）：每个被更新的点再取 `min(V, V(t=0))`；numpy 版在本地验证过，和整体 BRT 求解**符号完全一致**（L 形两条线：并集漏 74 格，local update 后 0 格）；fast-brt 新增 `target_set_mode="brt"` 和 `local_update_impl="numpy"`。原因和改动见 [12](12_local_update_brt.md)；
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

1. ~~**local update 和 BRT**~~：已解决（2026-11-21）。只把最后一层换成 BRT 不等价，所以改成在 local update 的每一步里取 `min(V, ℓ)`，见 [12](12_local_update_brt.md)。
2. **local update 的定位**：在线每周期跑，还是按事件触发、异步更新？numpy 版够不够快？
3. **6D 无人机用哪个模型**。
