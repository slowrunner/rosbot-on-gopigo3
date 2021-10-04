#!/bin/bash

echo "KILL_1k Started"

# ~/rosbot-on-gopigo3/handsonros2/install/setup.bash
echo "Killing gopigo3_node"
killall gopigo3_node

# echo "Killing distance_sensor"
# killall distance_sensor

# echo "Killing ultrasonic_ranger"
# killall ultrasonic_ranger

echo "Killing imu_sensor"
killall imu_sensor

# echo "Trying killall  ydlidar_ros2_driver_node"
# killall ydlidar_ros2_driver_node

echo "Trying killall teleop_node and joy_node"
killall ros2
killall joy_node
killall teleop_node

echo "Killing snapJPG and raspistill"
kill $(ps aux | grep '[p]ython3 snapJPG.py' | awk '{print $2}')
killall raspistill

echo "kill_1k.sh Completed"
