#!/bin/bash

# ~/rosbot-on-gopigo3/handsonros2/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/Twist 'linear: {x: 0}'

