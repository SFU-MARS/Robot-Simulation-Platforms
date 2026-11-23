# 12 local update 的 BRT 版本：为什么要改 HJ_local_update

> 日期：2026-11-22。代码：HJ_local_update `robot-sim` 分支 `843950e`，fast-brt `robot-sim` 分支 `dddf771`（两个 `main` 都没动）。
> 动力学：恒速 Dubins 3D。

## 12.1 结论

1. **避障必须用 BRT**（t ∈ [0, T]），不能用 BRS（t = T）。BRS 会把"中途撞墙、到 T 时已经穿过去"的状态判为安全。精度复核里，BRS 在面正前方的高度是 **0**（[10 §10.1](10_primitive_alpha_accuracy.md)）。
2. **BRS 算完以后，没办法事后补成 BRT**：把各时刻的 BRS 取并集，或者只替换最后一层，得到的集合都比 BRT **小**，误差方向是不安全的。`min(V, ℓ)` 必须放进**每一个时间步的递推里**（odp 的 `minVWithV0`）。
3. 原来 fast-brt 的**整条流程都是 BRS**：primitive library、整体求解、HJ_local_update 都用 `TargetSetMode "None"`。流程本身是自洽的，但不能用来避障。
4. 改成 BRT 时，primitive library 只要改 config。但 **local update 是在区域内自己一步步重算 HJ**，所以 kernel 本身也要改：每一步之后再取 `min(V, V(t=0))`。
5. numpy 版已经验证，和整体 BRT 求解的**符号完全一致**：L 形两条线，并集判错 74 格，local update 之后 0 格。HeteroCL 版还要在 mars-14 上验证。
6. 新行为默认关闭（`brt=False`、`target_set_mode="brs"`），师妹在 `main` 上的结果不受影响。

## 12.2 BRS 和 BRT 差在哪

记 `ℓ(x)` 为障碍的初始值（`ℓ < 0` 表示在障碍里），`ξ(t)` 为从 x 出发、控制为 u 的轨迹。

| | 定义 | value |
| --- | --- | --- |
| BRS（t = T） | 不管怎么控制，**恰好在时刻 T** 处在障碍里 | `V_BRS(x, T) = max_u ℓ(ξ(T))` |
| BRT（t ∈ [0, T]） | 不管怎么控制，**在 [0, T] 里某个时刻**会碰到障碍 | `V_BRT(x) = max_u min_{t ≤ T} ℓ(ξ(t))` |

**例子：正对一面薄墙。** primitive 是长 L、厚 `L/√α` 的矩形；L = 1 R、α = 20 时，厚度约 0.22 R。恒速车正对着墙开过去：
- 动力学里障碍不会让车停下，车约 0.22 个时间单位就"穿过"了墙；
- 到 T（horizon ≥ π/2）时，车已经在墙背后，`ℓ(ξ(T)) > 0`；
- 所以它**不在 BRS 里**，被判为安全，但它中途明明撞上了。BRT 会把它算进去。

这就是 BRS 在面正前方高度为 0 的原因。primitive 越薄，这个问题越明显。

## 12.3 为什么不能事后补救

一个自然的想法是：已经有各时刻的 BRS，取并集（对时间取 min）就是 BRT；或者只把最后一层换掉。两个都不对，因为**量词顺序不同**：

$$
V_{\text{BRT}}(x) = \max_{u}\ \min_{t \le T} \ell(\xi(t)), \qquad
\min_{t \le T} V_{\text{BRS}}(x, t) = \min_{t \le T}\ \max_{u}\ \ell(\xi(t)) \ \ge\ V_{\text{BRT}}(x)
$$

- 左边要求**同一个控制**在所有时刻都避开障碍。
- 右边允许**每个时刻各用一个控制**：只要求"对每个固定的 t，都存在某个控制让 t 时刻不在障碍里"。

所以右边更大，BRS 拼出来的集合只会比 BRT **小**。

正确做法是在 HJ 递推的**每一步**都和 `ℓ` 取 min，也就是 odp 的 `TargetSetMode "minVWithV0"`：

$$
V^{k+1} = \min\big(\,V^{k} + \Delta t \cdot H(V^{k}),\ \ \ell\,\big), \qquad V^{0} = \ell
$$

这样每一步取 min 之后得到的值，都已经是"到这一步为止有没有碰过"的结果，后面的步再从这里往前推。

## 12.4 local update 是怎么工作的，为什么也得改

fast-brt 每次 `compute()` 的流程（[09 §9.2](09_fast_brt_integration.md)）：

1. **place**：每个障碍放一个 primitive；value 从 library 里取，library 存了每个时间步。
2. **union**：所有 primitive 的 value 取 min，得到 `result_decomp`（每个时间步一层）。它比真正的整体解偏"安全"：每个 primitive 单独都能躲开，合在一起躲不开的地方就是 leak，多半在拐角。
3. **leaking set**：最小值和次小值相遇的点。
4. **local update**：从 t = 0 开始逐步重算。
   - 每一步只在一个区域里重算一步 HJ，这个区域是 leaking set 加上一步被修正过的点的邻居；
   - 区域外直接用 `result_decomp` 的值。

第 4 步是 HJ_local_update **自己写的 HJ 时间步**（numpy 版和 HeteroCL kernel），不调用 odp 的 solver。所以：

- 如果 library 是 BRT，而 local update 每一步还按 BRS 推进，**区域内算出来的是 BRS**，又回到 12.2 的问题。
- 区域内外的值也对不上：外面是 BRT，里面是 BRS，接缝处会产生假的差异。区域按"差异 > threshold"来扩张，这些假差异会让它越扩越大。

这和"格式必须一致"是同一个道理：library 用一阶（`accuracy="low"`），local update 的 kernel 也是一阶。两边哪怕差一点，修正区域都会扩散到整张网格。**BRT/BRS 和数值格式，library、整体求解、local update 三者必须完全一样。**

## 12.5 改动

只在每次单点更新之后加一行 `min`。`result_decomp[0]` 就是 t = 0 时所有 primitive 的初始值取 min，即合在一起的障碍 `ℓ`。

| 位置 | 改动 |
| --- | --- |
| [`numpy_impl/growing_region.py`](../fast-brt/HJ_local_update/numpy_impl/growing_region.py#L66) | `local_update_from_file(..., brt=False)`：`if brt: new_data[index] = min(new_data[index], result_decomp[0][index])` |
| [`hcl_impl/growing_region.py`](../fast-brt/HJ_local_update/hcl_impl/growing_region.py#L159) | `build_local_update_kernel(..., brt=False)`：`if brt: with hcl.if_(V[(0,) + idx] < new[0]): new[0] = V[(0,) + idx]`；`brt` 加进 kernel 缓存的 key，BRS 和 BRT 的 kernel 分开缓存 |
| [`compute/hj_solver_wrapper.py`](../fast-brt/HJ_local_update/compute/hj_solver_wrapper.py) | `solve_hj(..., target_set_mode="None")`：可以传 `"minVWithV0"`，用于 BRT 的 library 和整体求解 |
| fast-brt [`config.py`](../fast-brt/fast_brt/config.py) | 新字段 `target_set_mode`（`"brs"` 默认 / `"brt"`）和 `local_update_impl`（`"hcl"` 默认 / `"numpy"`），以及 `solve_hj_options()` |
| fast-brt [`library.py`](../fast-brt/fast_brt/library.py)、[`core/verify.py`](../fast-brt/fast_brt/core/verify.py) | 预计算 library 和整体求解都用同一个 `target_set_mode`；`target_set_mode` 记进 library 的 `config.json`，BRS 的 library 不能拿来做 BRT（会报错） |
| fast-brt [`core/local_update.py`](../fast-brt/fast_brt/core/local_update.py#L48) | 只在 BRT 时传 `brt=True`，所以 fast-brt 也能配合没有这个参数的上游 HJ_local_update |

**怎么打开：**

```python
config = Config.from_json("configs/dubins_3d.json")
config.target_set_mode = "brt"          # library、整体求解、local update 都用 BRT
config.local_update_impl = "numpy"      # 可选：不用 HeteroCL
```

BRT 需要**重新预计算 library**。已有的 BRS library 在 BRT config 下加载时会报错，不会被悄悄用错。

## 12.6 验证

**已完成（laptop，numpy 版）：** Dubins 3D，两条线组成 L 形，和"所有障碍放在一起直接求 BRT"（同样一阶、BRT）对比：

| | 判错的格子（符号和整体解不同） | 集合附近最大误差 |
| --- | --- | --- |
| 只取 primitive BRT 的并集 | 74 | — |
| 并集 + BRT local update | **0** | 2.7e-3 |

**待做（mars-14，HeteroCL）：**

```bash
cd fast-brt
pytest -m slow -s -k tube                                   # test_avoid_local_update_matches_direct_tube
python experiments/local_update_timing.py                   # brs/brt × 2–4 条线 × 3D/4D，计时 + 和整体解对比
python experiments/local_update_timing.py --impl numpy --dims 3 --lines 2
```

测试检查三件事：
- 判错格子比并集少；
- 集合附近的误差在阈值内；
- 总耗时不超过 look-back 时长的 25%。

BRT 判错格子的阈值等第一批结果出来再定。`local_update_timing.py` 还会输出总耗时和 Burger 控制周期 0.2 s 的比值。

## 12.7 还没解决的

- **只能用一阶格式。** local update 的 kernel 是一阶 ENO + forward Euler，所以它用的 library 必须是 `accuracy="low"`。但给 filter 推荐的离线 `V₀` 是二阶（`"medium"`，[10 §10.3](10_primitive_alpha_accuracy.md)），一阶在面前方平均低估约 0.22 R。可选的方向：
  - 换更细的网格加线性编码，减小一阶的误差；
  - 给 kernel 写二阶版本（ENO2 + TVD-RK2）；
  - 混合：filter 用二阶的 `V₀`，再叠加一阶 local update 算出来的修正量，例如 `U_high + min(0, Δ_low)`。
- **两套 library。** 在此之前，filter 和 local update 暂时各用一套 library。
- **实时性。** HeteroCL 和 numpy 各有多快、是每个周期都跑还是按检测事件异步跑，要等 mars-14 的计时结果（[11 §11.6](11_roadmap.md)）。
