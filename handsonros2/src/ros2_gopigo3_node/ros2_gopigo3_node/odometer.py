#!/usr/bin/env python

# FILE: odometer.py

"""
   Record distance traveled

   Subscribes: /odom

   Starts a recording a segment when motion detected
   Continues adding distance traveled until motion ends
   Logs datetime and distance traveled

   nav_msgs.msg/Odometry msg consists of:
      std_msgs/Header                    header
      string                             child_frame_id
      geometry_msgs/PoseWithCovariance   pose  (geometry_msgs/Point, geometry_msgs/Quaternion)
      geometry_msgs/TwistWithCovariance  twist
"""
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry    # (header, child_frame_id, pose, twist)
from geometry_msgs.msg import Point  # (position: float64 x,y,z)
import logging
import datetime as dt
from rclpy.time import Time

class OdometerNode(Node):

  last_point = Point()
  current_point = Point()
  start_timestamp = Time()    # (int32 sec, uint32 nanosec)
  current_timestamp = Time()    # header.stamp (int32 sec, uint32 nanosec)
  last_timestamp = Time()

  moving = False
  moved_x = 0.0
  total_x = 0.0
  startup = True
  ODOLOGFILE = '/home/pi/rosbot-on-gopigo3/odom.log'

  # start_time =

  def __init__(self):
    super().__init__('odometer')
    # self.pub = self.create_publisher(Int32, 'doubled', qos_profile=10)
    self.sub = self.create_subscription(
      Odometry,
      'odom',
      self.sub_callback,
      10)
    self.sub  # prevent unused var warning
    self.get_logger().info('odometry topic subscriber created')

    self.odoLog = logging.getLogger('odoLog')
    self.odoLog.setLevel(logging.INFO)

    self.loghandler = logging.FileHandler(self.ODOLOGFILE)
    self.logformatter = logging.Formatter('%(asctime)s|%(message)s',"%Y-%m-%d %H:%M:%S")
    self.loghandler.setFormatter(self.logformatter)
    self.odoLog.addHandler(self.loghandler)
    # self.get_logger().info('doubled topic publisher created')


  def sub_callback(self,odometry_msg):
    # segment_msg = ()
    self.current_point = odometry_msg.pose.pose.position
    self.current_timestamp = odometry_msg.header.stamp
    # self.get_loger().info(odom_msg.pose.pose)

    if self.startup:
        self.last_point.x = self.current_point.x
        self.last_timestamp = self.current_timestamp
        self.startup = False

    if math.isclose(self.current_point.x, self.last_point.x, abs_tol=0.00001):
        if (self.moving == True):   # end of motion
            self.total_x += abs(self.moved_x)
            moving_seconds = self.current_timestamp.sec + (self.current_timestamp.nanosec/1000000000.0) - self.start_timestamp.sec - (self.start_timestamp.nanosec/1000000000.0)

            printMsg = "current_point - x: {:.3f} y: {:.3f} z: {:.3f} - moved: {:.3f} total moved: {:.3f} in {:.1f}s".format(
                       self.current_point.x, self.current_point.y, self.current_point.z, self.moved_x, self.total_x, moving_seconds)
            print(printMsg)
            print("stopped moving")

            # Log this travel segment to odom.log
            logMsg = "travel: {:>7.3f}  motion: {:>7.1f} sec".format(self.moved_x, moving_seconds)
            self.odoLog.info(logMsg)

            self.moving = False
        else:   # was not moving and still not moved
            pass
    else:  # moving
        if (self.moving == False):  # start of motion
            print("started moving")
            self.moving = True
            self.moved_x = self.current_point.x - self.last_point.x
            print("current_point - x: {:.3f} y: {:.3f} z: {:.3f} - moved: {:.3f}".format(self.current_point.x, self.current_point.y, self.current_point.z, self.moved_x))
            self.start_timestamp = self.last_timestamp

        else:     # still moving
            self.moved_x += self.current_point.x - self.last_point.x
            print("current_point - x: {:.3f} y: {:.3f} z: {:.3f} - moved: {:.3f}".format(self.current_point.x, self.current_point.y, self.current_point.z, self.moved_x))

    self.last_point = self.current_point
    self.last_timestamp = self.current_timestamp
    if self.moving == False:
        self.moved_x = 0.0
    # self.pub.publish(doubled_msg)


def main(args=None):

  rclpy.init(args=args)
  odometer_node = OdometerNode()
  try:
    rclpy.spin(odometer_node)
  except KeyboardInterrupt:
    print('\ncontrol-c: odometer_node shutting down')
  finally:
    odometer_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
  main()


