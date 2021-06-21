#!/usr/bin/env python3
#
# File: easy_square.py
# Results:  The GoPiGo3 will
#    announce moving in 5 seconds
#    move forward for 18 inches, pause 5 seconds
#    turn 90 right, pause 5 seconds
#    drive forward 12 inches, pause 5 seconds
#    turn 90 right, pause 5 seconds
#    drive forward 12 inches, pause 5 seconds
#    turn 90 right, pause 5 seconds
#    drive forward 12 inches, pause 5 seconds
#    turn 90 right, pause 5 seconds
#    drive backward 6 inches
#


# import the time library for the sleep function
import time
import sys
import math

# import the GoPiGo3 drivers
from easygopigo3 import EasyGoPiGo3 

# Create an instance of the GoPiGo3 class.
# GPG will be the GoPiGo3 object.
egpg = EasyGoPiGo3(use_mutex=True)

egpg.WHEEL_DIAMETER = 66.77
egpg.WHEEL_BASE_WIDTH = 106.14    # 106.14
egpg.WHEEL_BASE_CIRCUMFERENCE = egpg.WHEEL_BASE_WIDTH * math.pi # The circumference of the circle the wheels will trace while turning (mm)
egpg.WHEEL_CIRCUMFERENCE      = egpg.WHEEL_DIAMETER   * math.pi # The circumference of the wheels (mm)
egpg.ENCODER_TICKS_PER_ROTATION = 16  # MAGIC NUMBER TO MAKE NEW GOPIGO3 KIT WORK
egpg.MOTOR_TICKS_PER_DEGREE = ((egpg.MOTOR_GEAR_RATIO * egpg.ENCODER_TICKS_PER_ROTATION) / 360.0) # encoder ticks per output shaft rotation degree

print("egpg.WHEEL_DIAMETER: ", egpg.WHEEL_DIAMETER)
print("egpg.WHEEL_BASE_WIDTH: ", egpg.WHEEL_BASE_WIDTH)
print("egpg.ENCODER_TICKS_PER_ROTATION: ", egpg.ENCODER_TICKS_PER_ROTATION)

egpg.set_speed(150)

print("SQUARE TEST will begin in 5 seconds")
time.sleep(5)

print("Driving forward 18 inches")
egpg.drive_inches(18.0)
time.sleep(5)

print("Turning Right 90 degrees")
egpg.turn_degrees(90.0)
time.sleep(5)

print("Driving forward 12 inches")
egpg.drive_inches(12.0)
time.sleep(5)

print("Turning Right 90 degrees")
egpg.turn_degrees(90.0)
time.sleep(5)

print("Driving forward 12 inches")
egpg.drive_inches(12.0)
time.sleep(5)

print("Turning Right 90 degrees")
egpg.turn_degrees(90.0)
time.sleep(5)

print("Driving forward 12 inches")
egpg.drive_inches(12.0)
time.sleep(5)

print("Turning Right 90 degrees")
egpg.turn_degrees(90.0)
time.sleep(5)

print("Driving backward 6 inches")
egpg.drive_inches(-6.0)


print("SQUARE TEST: Done!")

