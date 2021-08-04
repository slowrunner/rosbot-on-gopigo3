#!/bin/bash

cd ~/rosbot-on-gopigo3/handsonros2
. ~/rosbot-on-gopigo3/handsonros2/install/setup.bash
colcon build --packages-select ros2_gopigo3_node
colcon build --symlink-install --packages-select ydlidar_ros2_driver
. ~/rosbot-on-gopigo3/handsonros2/install/setup.bash

