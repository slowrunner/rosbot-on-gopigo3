# rosbot-on-gopigo3/handsonros2
GoPiGo3 ROSbot code - ROS2 versions of "Hands-On-ROS-For-Robotics-Programming"

Originals: https://github.com/PacktPublishing/Hands-On-ROS-for-Robotics-Programming


NOTE: "ROS2 Migration" 
- is one-to-one rospy-to-rclpy conversion
  with addition of a thread to execute rclpy.spin()  
  which is needed in ROS2 to make callbacks and rate.sleep() happen
  
"Pythonic ROS2 Migration"
- involves reorganization into a Class with main()
  of the one-to-one rospy-to-rclpy conversions


# USAGE:
- ./runit.sh  starts gopigo3 node, ydlidar node, distance sensor node, SNES Gamepad node  
  (ctrl-c to finish, then run ./killit.sh to finish off the rest)  
- ./killit.sh kills off distance sensor node and gopigo3 node  


# SNES Gamepad (/opt/ros/foxy/share/teleop_twist_joy/config/snes.config.yaml)
- ENABLE: Left Front Button  
- TURBO_ENABLE: Right Front Button  

- PAD UP/DN + ENABLE:  +0.1 / -0.1 m/s  
- PAD L/R + ENABLE: CCW 1.0 / CW 1.0 rad/s  

- PAD UP/DN + TURBO_ENABLE:  +0.19 / -0.19 m/s  (Any higher - Turbo Fwd curves right)
- PAD L/R + TURBO_ENABLE: CCW 0.25 / CW 0.25 rad/s  


