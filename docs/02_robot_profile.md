# 02 Robot profile 与单位换算

代码：[`nav_core/robots/profile.py`](../nav_core/nav_core/robots/profile.py)，配置：[`robots/*.yaml`](../robots/)

## 2.1 profile 是什么

每个机器人一个 YAML，是该机器人**所有参数的唯一来源**。代码里任何地方要知道"速度多大"、"lidar 看多远"、"地图多大"，都从 profile 读，不写死。

```yaml
# robots/burger.yaml
name: burger
status: ready                    # ready | placeholder
dynamics: dubins3d               # FaST-BRT 里的模型名
phys:
  v: 0.1                         # m/s，恒定前进速度
  w_max: 1.0                     # rad/s，转弯半径 v / w_max = 0.1 m
  track_width: 0.160             # m，轮距（differential drive）
  wheel_v_max: 0.22              # m/s，单轮速度上限
footprint_radius: 0.124          # m，以轮轴中心（位姿）为圆心、包住整台车的圆：14 cm 的底盘中心在轴后 3.2 cm，后角离轴心 0.1237 m
sensor:
  type: lidar2d                  # LDS-01
  range: 0.8                     # m，软件截断（硬件能到 3.5 m）
  range_min: 0.12                # m，LDS-01 最小量程
  height: 0.17                   # m，扫描平面离地高度
  fov: 6.283185307179586         # rad
  n_rays: 360
  rate_hz: 5.0                   # 实机 LDS-01 的频率
ctrl_dt: 0.2                     # s，一个 scan 一个 control period
map:
  origin: [-1.05, -1.05]         # m，地图左下角（墙在地图内）
  size: [2.1, 2.1]               # m
  resolution: 0.02               # m / cell → 105 × 105 格
world: turtlebot3/turtlebot3_controls/hj_reach_target.world
base_value: null                 # 离线算好的 value 文件（待填）
```

加载：

```python
from nav_core.robots import load_profile
p = load_profile("burger").require_ready()
```

`require_ready()` 对 placeholder 机器人直接抛异常，避免拿着一堆 `null` 往下跑。Crazyflie 现在是 placeholder。QBot（`dynamics: dubins4d`）从 2026-11-23 起是 ready：`phys` 写 `v_min`、`v_max`、`a_max`、`w_max`，单位换算加了速度和加速度（`speed_to_model`、`accel_to_model`），`profile.model` 是车的模型（`robots/dynamics.py` 的 `Dubins3D` / `Dubins4D`），见 [15](15_qbot_dubins4d.md)。

`sensor.height` 的用处：仿真时从 SDF world 里**只取穿过扫描平面的 collision box**，地板和 goal 贴纸会被自动过滤掉，障碍和 Gazebo 里 lidar 看到的完全一致。

## 2.2 单位换算：physical ↔ model

FaST-BRT 的 Dubins 3D 模型是**无量纲**的：`v = 1`，`|ω| ≤ 1`，所以转弯半径恒等于 1。

真实机器人速度 `v`、转向上限 `w_max`，那么：

```
1 个模型长度单位 = v / w_max 米   （正是转弯半径）
1 个模型时间单位 = 1 / w_max 秒
```

Burger（v = 0.1，w_max = 1.0）：**1 单位 = 0.1 m，1 单位 = 1.0 s**。

```python
U = p.units                      # Units(length=0.1, time=1.0)
U.pos_to_model([0.2, 0.4])       # -> [2.0, 4.0]
U.rate_to_model(0.5)             # rad/s -> rad / 模型时间单位
```

所有换算集中在 `Units` 里，`FastBrtGatekeeper` 是唯一调用它的地方（见 [05](05_safety_filter.md)）。其它 dynamics 暂时返回单位换算 `Units()`（1:1），等模型定下来再补。

**为什么要无量纲**：FaST-BRT 只需要解一次 value function，所有机器人、所有尺度共用。换个速度只是换个换算系数，不用重算 HJ。

## 2.3 运动能力检查 `check_limits()`

这是一个踩过的坑。TurtleBot3 Burger 的规格表写着最大线速度 0.22 m/s、最大角速度 2.84 rad/s，但**这两个数是分开测的，不能同时取到**：

- 2.84 rad/s 是原地旋转（v = 0）时的值；
- 0.22 m/s 是直线（ω = 0）时的值。

差速驱动的真实约束是**外侧轮速不超限**：

```
v + w_max × track_width / 2 ≤ wheel_v_max
```

| 设置 | 外侧轮速 | 可行？ |
| --- | --- | --- |
| v=0.2, ω=2 | 0.36 m/s | ✗ |
| v=0.2, ω=1 | 0.28 m/s | ✗ |
| **v=0.1, ω=1**（当前） | **0.18 m/s** | ✓ |

`load_profile` 在加载 ready 的 profile 时自动检查，不满足直接 `ValueError`。

真正决定能不能在场地里避障的是**转弯半径 R = v / w_max**，不是 v 和 ω 各自的大小。Dubins 车的几何轨迹与时间尺度无关：速度减半，路径形状完全不变，只是走得慢一倍。所以 Burger 选 v=0.1 / ω=1，保持 R = 0.1 m，同时满足轮速约束。

**为什么 R 不能是 0.2 m**：2 m 见方的场地里，R=0.2 m 时 loiter circle 加车身要约 0.65 m 的空地，起点处第一帧扫不出这么大的自由区域，gatekeeper 直接 `init_failed`，机器人不出发。

## 2.4 为不同机器人选参数

| 参数 | 怎么定 |
| --- | --- |
| `v`, `w_max` | 由 dynamics 和执行器上限定；先保证 `check_limits()` 通过 |
| 转弯半径 `v/w_max` | 必须明显小于场地最窄通道的一半，否则 filter 不让进 |
| `sensor.range` | 至少 ≈ `footprint_radius + 2 × 转弯半径 + 余量`，否则 `unknown="occupied"` 启动不了（Burger 约 0.34 m） |
| `map.resolution` | 决定长度估计精度（约 1 个 cell）和 clearance（默认 1.5 cell） |
| `ctrl_dt` | 和 lidar 频率对齐；Burger 的 LDS 是 5 Hz，所以 0.2 s |

`sensor.range` 这一条还没做成启动检查，在 TODO 里。
