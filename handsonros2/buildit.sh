#!/bin/bash

cd ~/rosbot-on-gopigo3/handsonros2
. ~/rosbot-on-gopigo3/handsonros2/install/setup.bash
colcon build --packages-select ros2_gopigo3_node
. ~/rosbot-on-gopigo3/handsonros2/install/setup.bash

