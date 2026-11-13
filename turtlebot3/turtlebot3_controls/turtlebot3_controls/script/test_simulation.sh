ign service -s /world/hj_reach_target/set_pose \
  --reqtype ignition.msgs.Pose \
  --reptype ignition.msgs.Boolean \
  --timeout 3000 \
  --req 'name: "burger" position: {x: -0.8, y: -0.8, z: 0.01} orientation: {w: 0.9950041653, z: 0.0998334166}' &> /dev/null

conda activate odp &> /dev/null
python3 reach_avoid.py >/dev/null

wc -l dubins_control.txt 

cd ../../../ &> /dev/null
export TURTLEBOT3_MODEL=burger &> /dev/null
source install/setup.bash &> /dev/null
ros2 run turtlebot3_controls controls_keyboard &> /dev/null 






