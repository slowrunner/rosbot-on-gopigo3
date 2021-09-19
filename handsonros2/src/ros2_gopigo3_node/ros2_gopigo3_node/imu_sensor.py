#!/usr/bin/env python3

# File: imu_sensor.py

# Basic ROS2 Node to publish imu sensor readings using Alan's SafeIMUSensor() class
"""
  Publishes:
     /imu/data
     /imu/mag
     /imu/temp
  TODO:
     /imu/status
     /imu/raw
"""
import rclpy
from rclpy.node import Node
# from angles import from_degrees
from sensor_msgs.msg import Imu, Temperature, MagneticField
from std_msgs.msg import Header
# from diagnostic_msgs import DiagnosticStatus, DiagnosticArray, KeyValue
from imu4gopigo3ros import SafeIMUSensor
import math


class IMUSensorNode(Node):

    def __init__(self):
        super().__init__('imu_sensor')
        self.imu = SafeIMUSensor(use_mutex=True)
        self.msg_imu = Imu()
        self.msg_temp = Temperature()
        self.msg_magn = MagneticField()
        # TODO: self.msg_diag = DiagnosticStatus()

        # setting frame_id to urdf link element "imu_link"
        self.hdr = Header(stamp=self.get_clock().now().to_msg(), frame_id="imu_link")  # use common time for all published topics

        self.pub = self.create_publisher(Imu, '~/imu', qos_profile=10)
        self.pub = self.create_publisher(Temperature, '~/temp', qos_profile=10)
        self.pub = self.create_publisher(MagneticField, '~/magnetometer', qos_profile=10)
        # TODO: self.pub = self.create_publisher(DiagnosticStatus, '~/status', qos_profile=10)
        timer_period = 0.03333  #  second = 30 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Created imu_sensor node at {:.0f} hz'.format(1.0/timer_period))

    def timer_callback(self):
        # Following Hands-On-ROS, performs multiple reads - perhaps should be reading all at once?

        self.q = self.imu.safe_read_quaternion()         # x,y,z,w
        self.mag = self.imu.safe_read_magnetometer()     # micro Tesla (µT)
        self.gyro = self.imu.safe_read_gyroscope()       # deg/second
        self.accel = self.imu.safe_read_accelerometer()  # m/s²
        self.temp = self.imu.safe_read_temperature()     # °C
        self.cal  = self.imu.safe_sgam_calibration_status() # sysCal, gyroCal, accCal, magCal none:0 - fully:3


        self.hdr.stamp = self.get_clock().now().to_msg()  # save common time stamp for all topics

        self.msg_temp.header = self.hdr
        self.msg_temp.temperature = self.temp
        self.pub.publish(self.msg_temp)
        # self.get_logger().info('Publishing: {}'.format(self.msg_range))


def main(args=None):
    rclpy.init(args=args)

    imu_node = IMUSensorNode()

    try:
      rclpy.spin(imu_node)
    except KeyboardInterrupt:
      print('\ncontrol-c: imu_sensor node shutting down')
    finally:
      # Destroy the node explicitly
      # (optional - otherwise it will be done automatically
      # when the garbage collector destroys the node object)
      imu_node.destroy_node()
      rclpy.shutdown()


if __name__ == '__main__':
    main()
