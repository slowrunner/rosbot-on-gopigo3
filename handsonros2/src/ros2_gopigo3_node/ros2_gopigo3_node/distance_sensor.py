#!/usr/bin/env python3

# File: distance_sensor.py

# Basic ROS2 Node to publish distance sensor readings

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from di_sensors.easy_distance_sensor import EasyDistanceSensor



class DistanceSensorNode(Node):

    def __init__(self):
        super().__init__('distance_sensor')
        self.ds = EasyDistanceSensor(use_mutex=True)
        self.pub = self.create_publisher(Float32, '~/distance', qos_profile=10)
        timer_period = 0.5  # 0.5 seconds = 2Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
        self.get_logger().info('Created distance_sensor node')

    def timer_callback(self):
        self.reading = self.ds.read()
        print("distance reading: {} cm".format(self.reading))
        msg = Float32()
        msg.data = float(self.reading)
        self.pub.publish(msg)
        self.get_logger().info('Publishing: {}'.format(msg.data))


def main(args=None):
    rclpy.init(args=args)

    ds_node = DistanceSensorNode()

    try:
      rclpy.spin(ds_node)
    except KeyboardInterrupt:
      print('\ncontrol-c: distance_sensor node shutting down')
    finally:
      # Destroy the node explicitly
      # (optional - otherwise it will be done automatically
      # when the garbage collector destroys the node object)
      ds_node.destroy_node()
      rclpy.shutdown()


if __name__ == '__main__':
    main()
