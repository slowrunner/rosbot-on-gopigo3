#!/bin/bash

cd ~/rosbot-on-gopigo3/handsonros2
. ~/rosbot-on-gopigo3/handsonros2/install/setup.bash

# start SNES gamepad node (cntrl-c to stop )
echo "Starting SNES Gamepad teleop_twist_joy - cntrl-c to quit"
ros2 launch teleop_twist_joy teleop-launch.py joy_config:="snes" 
