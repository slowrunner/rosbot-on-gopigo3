#!/bin/bash

cd ~/rosbot-on-gopigo3/handsonros2
. ~/rosbot-on-gopigo3/handsonros2/install/setup.bash
# ros2 run ros2_gopigo3_node gopigo3_node &
ros2 run ros2_gopigo3_node gopigo3_node --ros-args -p S1LPW:=2094 -p S1RPW:=750 -p S1SECTOR:=2.443 &
# ros2 run ros2_gopigo3_node gopigo3_node --ros-args --params-file ./src/ros2_gopigo3_node/gopigo3_node_params.yaml &
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
