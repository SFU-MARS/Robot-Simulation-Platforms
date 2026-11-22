# 08 实验结果

所有实验都在 `nav_core` 的 ROS-free 仿真里跑，可在 laptop 上复现。

## 8.1 nominal controller 对比

场景：随机起点 × 多个 goal。

| 场景 | nominal | 只用 nominal | 加 gatekeeper |
| --- | --- | --- | --- |
| `hj_reach_target`（26 次） | pursuit | 10 到达，**16 撞** | 13 到达，**0 撞**，13 超时，介入率 0.26 |
| | MPPI | 19 到达，**7 撞** | 13 到达，**0 撞**，13 超时，介入率 **0.09** |
| 墙 + 独立柱子（35 次） | pursuit | 22 到达，**13 撞** | 35 到达，**0 撞**，介入率 0.08 |
| | MPPI | 34 到达，**1 撞** | 35 到达，**0 撞**，介入率 **0.00**，最小距离 6.7 cm |

结论：

- **gatekeeper 在所有配置下都是 0 碰撞**；
- MPPI 是更好的 nominal：不加 filter 时碰撞大减，加上后几乎不用 filter 出手；
- 但 MPPI 自己仍会撞，filter 仍然必要；
- 计算开销：pursuit 约 2–3 ms/step，MPPI 约 5 ms/step，加 gatekeeper 后 18–35 ms/step，都远小于 `ctrl_dt = 200 ms`。

### `hj_reach_target` 的 13 次超时

换成 MPPI 也没减少。原因查清了：该场景左上角只能通过 obstacle 1 和北墙之间一条 **0.4 m** 宽的缝进去，而 loiter circle 需要约 0.45 m。26 次实验里恰好 **13 次**需要过这条缝，数字完全对上。

这是 constant-speed 模型的结构性限制，不是 nominal 或 filter 的 bug。能刹车的 Dubins4D 不会有。

## 8.2 两种 unknown 模式 × lidar 量程

场景：`occluded_4x4.world`（3 面挡墙，每面一端后藏一个箱子；3 根独立柱子），nominal 用 MPPI，每行 26 次。

| lidar 量程 | 不加 filter | `unknown=occupied` | `unknown=free` |
| --- | --- | --- | --- |
| 0.3 m（3 × 转弯半径） | 22 到达，4 撞 | **26 次全部 init_failed** | 21 到达，**5 撞** |
| 0.5 m（5 ×） | 24 到达，2 撞 | 19 到达，0 撞，7 超时 | 20 到达，0 撞，6 超时 |
| 0.8 m（8 ×） | 22 到达，4 撞 | 20 到达，0 撞，6 超时 | 20 到达，0 撞，6 超时 |

结论：

1. **量程短时，`unknown=free` 不安全。** 它把没看到的地方当可走，于是 filter 会放行一条通往"可能藏着障碍"的路。量程 0.3 m 时箱子被看到得太晚，来不及转开。
2. **`unknown=occupied` 从不碰撞，但有量程下限。** 第一帧扫出的自由空间必须能放下 loiter circle，大约需要 `footprint + 2 × 转弯半径 + 余量 ≈ 0.34 m`。量程 0.3 m 时它直接拒绝出发，这是**安全的失败方式**。
3. **量程 ≥ 5 倍转弯半径时两种模式都安全**，而且 `free` 并没有更快。

所以默认用 `occupied`。`free` 保留用于对比，只有在能证明"障碍进入 BRT 之前一定被看到"时才可用。

## 8.3 primitive 来源对比

`faces`（障碍层的包裹面）vs `rings`（region 的边）：

| 场景 | primitive | 到达 | 碰撞 | 超时 | 最小距离 | 每步耗时 |
| --- | --- | --- | --- | --- | --- | --- |
| hj（26 次） | rings | 13 | 0 | 13 | 3.4 cm | 14–18 ms |
| | **faces** | 13 | 0 | 13 | **5.5 cm** | 16–17 ms |
| 柱子（35 次） | rings | 35 | 0 | 0 | 2.5–3.1 cm | 27–30 ms |
| | **faces** | 35 | 0 | 0 | **4.8 cm** | 28–29 ms |

安全性和到达率相同，`faces` 离障碍更远（包裹出来的面、方拐角，略保守），开销基本不变。两种 unknown 模式下结论一致。

## 8.4 地图更新模型对比

场景：遮挡场景 3 条路线。指标"flicker"= 障碍格子数回落的次数（静态场景里障碍只应变多）。

| 地图模型 | 无噪声 | 1 cm | 2 cm | 空地假障碍格子（2 cm） | face 最深切进真实障碍（1 / 2 cm） |
| --- | --- | --- | --- | --- | --- |
| log-odds（旧默认） | 38 | 80 | 112 | 0 | 3.0 / 2.0 cm |
| log-odds + graze | 25 | 61 | 76 | 0 | 3.0 / 1.5 cm |
| counts τ=0.1 | 0 | 6 | 23 | 31 | 0 / 4.5 cm |
| **counts τ=0.2 + graze（现默认）** | **2** | **15** | **27** | 5 | **0.3 / 0.6 cm** |
| counts τ=0.3 | 6 | 19 | 35 | 0 | 3.0 / 1.0 cm |

选 τ=0.2 + graze 的理由：flicker 降到旧模型的 1/4 以下，同时 face 切进真实障碍最浅（安全性最好），空地假障碍也很少。

切换默认模型后重跑了 §8.1–8.3 的全部实验：**所有配置仍然 0 碰撞**，只有遮挡场景的两格数字各变了 1–2 次 episode。

## 8.5 障碍长度估计精度

`hj_reach_target`，机器人走完一圈后：

| | 结果 |
| --- | --- |
| 障碍个数 | 1（两个 obstacle 和墙连在一起，按设计决策算一个） |
| face 个数 | 12 |
| convex corner | 4 个，**正好是 obstacle 1 和 2 的 4 个外角** |
| obstacle 1 四边估计 | 0.39 / 0.42 / 0.50 / 0.53 m（真实 0.4 × 0.5 m） |
| 包裹性 | 所有 face 都在真实障碍外侧 ≥ 0.95 cm |
| 1 cm 噪声（每处 10 帧） | 13 个 face、5 个 convex corner，包裹仍成立（≥ 0.86 cm） |

遮挡场景里长度估计随观测单调变准的例子：

```
t=0.0s   obstacle 3 new:     0.91 m
t=4.6s   obstacle 3 updated: 1.40 m          ← 真实 1.4 m
t=8.6s   obstacle 4 new:     0.10 m（刚看到藏在墙后的箱子）
t=10.4s  obstacle 4 updated: 0.23 / 0.20 m   ← 真实 0.2 × 0.2 m
```

## 8.6 复现

```bash
cd nav_core

# 单次运行 + 出图 + 打印事件
PYTHONPATH=.:<FaST-BRT 路径> python scripts/run_sim.py --nominal mppi --filter gatekeeper --events

# 全部测试（43 个）
PYTHONPATH=. python -m pytest -q tests
```

batch 实验的脚本没有进 repo（都是几十行的一次性脚本），需要时按 §8.1 的配置重写即可：固定随机种子生成起点，对每种配置跑 `run_episode`，统计 `status`、最小距离、介入率。
