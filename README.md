# Robot Platforms


## Turtlebot3

### Setup 

Follow setup instructions here: https://docs.robotis.com/docs/systems/turtlebot3/simulation/gazebo_simulation


### Launch simulation

We are following the basic example from https://docs.robotis.com/docs/systems/turtlebot3/simulation/gazebo_simulation.


```
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Controlling the turtlebot can be done through this script: `turtlebot3/turtlebot3_controls/turtlebot3_controls/script/controls_keyboard.py`

```
colcon build --packages-select turtlebot3_controls
source install/setup.bash
ros2 run turtlebot3_controls controls_keyboard
```

## Mujoco Python simulator





