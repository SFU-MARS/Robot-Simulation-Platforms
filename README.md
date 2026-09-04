# Robot Platforms

This repository follows the [Fast-BRT Experiment Document](https://docs.google.com/document/d/1S5lzXlQSW35xeCZrHe30NN7NRfxiYE0QOhNOWc51FYE/edit?tab=t.0). The HJ computation is performed using `optimized_dp`. The output of `optimized_dp` **should** then be used to test on multiple robot simulation platforms. Most folders are submodules to other repositories. They must be explicitly cloned by adding the `--recurse-submodules` flag to a `git checkout` or `git clone`

## HJ Computation 

This computes the optimal control values for the 2D Dubins car (equivalent to turtle bot). The script should then output these commands into a text file for a robot simulation platform to read.

Follow the `optimized_dp` setup steps and setup the conda environment within this repository. The script can be run here:

```
cd turtlebot3/turtlebot3_controls/turtlebot3_controls/script
conda activate odp
python3 reach_avoid.py 
```


## Turtlebot3

Currently, the basic tutorial is present here. The outputs of `optimized_dp` will need to be used in the turtlebot simulation.

### Setup 

Follow setup instructions here: https://docs.robotis.com/docs/systems/turtlebot3/simulation/gazebo_simulation


cd turtlebot3
colcon build
source install/setup.bash`
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo baseline_fastbrt.launch.py
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





