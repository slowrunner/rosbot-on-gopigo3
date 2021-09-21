#!/bin/bash

# ~/rosbot-on-gopigo3/handsonros2/install/setup.bash
echo "Killing gopigo3_node"
killall gopigo3_node
echo "Killing distance_sensor"
killall distance_sensor
echo "Killing ultrasonic_ranger"
killall ultrasonic_ranger

echo "Don't know how to kill ydlidar_ros2_driver_node, remember to use cntl-c"

# Use when these are set up as lifecycle nodes
# ros2 lifecycle set gopigo3_node shutdown
# ros2 lifecycle set ydlidar_ros2_driver_node shutdown
