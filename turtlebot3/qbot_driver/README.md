# QBot Driver
Basic code for driving the QBot.

## Quickstart
1. Clone this repo into your workspace/src folder (e.g. `ros2_ws/src`)
2. Build it in your workspace folder (e.g. `ros2_ws`)
```bash
colcon build --symlink-install
source install/setup.bash
```
3. Launch the main launch file
```bash
ros2 launch qbot_driver bringup.launch.py
```
