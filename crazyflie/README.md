# Santiago Crazyflie simulation

Drone simulations using different methods:

- Basic dynamics: `goTo`
- Real dynamics: `cmdFullState`

Simulations are run in Docker and visualized with [Foxglove](https://foxglove.dev/).
- Watch it in the browser at [app.foxglove.dev](https://app.foxglove.dev) or download the [desktop app](https://foxglove.dev/download).
- Start the bridge, then connect Foxglove to `ws://localhost:8765`:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```
