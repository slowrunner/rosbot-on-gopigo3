#!/usr/bin/env python3
# Grove Ultrasonic Range Sensor example for the GoPiGo3
#
# Reads sensor data from the Grove Ultrasonic Range Sensor.
#
# The Grove Ultrasonic Ranger is compatible with the GoPiGo3 AD1 and AD2 ports.
# (Board has Grove connector and Grove pin-out.  Use Grove cable from sensor to GoPiGo3)
#
# Note: The HC-SR04 Ultrasonic Range Sensor is not compatible with the GoPiGo3


import time
from easygopigo3 import EasyGoPiGo3

# This example shows how to read values from the Distance Sensor
# The Grove Ultrasonic Distance Sensor is roughly accurate to 1cm (out to 40cm or so)
# There is a read_mm() but values appear to be low by 1-15mm)

# Create an instance of the EasyGoPiGo3 class.
# (GoPiGo3 or EasyGoPiGo3 object required because the sensor
#    is connected through the GoPiGo3 AD1 or AD2 ports)
egpg = EasyGoPiGo3()

print("Ignore initial value error msg")

# Create an instance of an UltraSonicSensor class
my_ultrasonic_sensor = egpg.init_ultrasonic_sensor(port="AD2")

try:
    while True:
        # read() returns the average of three measurements
        #     with values between 2 cm and 430 cm or 501 for "no detection"
        # print the value returned by the sensor.
        print("Ultrasonic Distance Sensor Reading: {} cm ".format(my_ultrasonic_sensor.read()),end='\r')
        time.sleep(0.06)  # read only every 60ms

except KeyboardInterrupt:
       print("\nCntl-C detected, Exiting")
