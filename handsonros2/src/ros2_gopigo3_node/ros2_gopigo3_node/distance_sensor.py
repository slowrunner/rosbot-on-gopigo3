#!/usr/bin/env python3

# File: distance_sensor.py

# Basic ROS2 Node to publish distance sensor readings

import rclpy
from rclpy.node import Node
# from angles import from_degrees
from sensor_msgs.msg import Range
from di_sensors.easy_distance_sensor import EasyDistanceSensor
import math


class DistanceSensorNode(Node):

    def __init__(self):
        super().__init__('distance_sensor')
        self.ds = EasyDistanceSensor(use_mutex=True)
        self.msg_range = Range()
        # setting frame_id to urdf link element "distance_sensor"
        self.msg_range.header.frame_id = "distance_sensor"
        self.msg_range.radiation_type = Range.INFRARED   # LASER is closer to INFRARED than ULTRASOUND
        self.msg_range.min_range = 0.0200   #         2 cm /   20 mm
        self.msg_range.max_range = 3.000    # 3 m / 300 cm / 3000 mm
        # self.msg_field_of_view = from_degrees(25.0)     # +/- 12.5 degree FOV (~60cm at max range)
        self.msg_range.field_of_view = math.radians(25.0)     # +/- 12.5 degree FOV (~60cm at max range)
        self.pub = self.create_publisher(Range, '~/distance', qos_profile=10)
        timer_period = 1.0  # 1 second = 1 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Created distance_sensor node at {:.0f} hz'.format(1.0/timer_period))

    def timer_callback(self):
        self.reading_mm = self.ds.read_mm()
        # print("distance reading: {} mm".format(self.reading_mm))
        self.msg_range.range = self.reading_mm/1000.0
        self.msg_range.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg_range)
        # self.get_logger().info('Publishing: {}'.format(self.msg_range))


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
