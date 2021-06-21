#!/usr/bin/python3

# FILE: myconfig.py
#
# PURPOSE: Set ROSbot on (new) GoPiGo3  specific parameters
#
# USAGE:
#   import sys
#   sys.path.append("/home/pi/rosbot-on-gopigo3/plib/")
#   import easygopigo3
#   import myconfig
#   egpg = easygopigo3.EasyGoPiGo3(use_mutex=True)
#   myconfig.setParameters(egpg,verbose=True)  # verbose default is False

from math import pi


# Wheel Dia and Wheel Base for ROSbot
WHEEL_DIAMETER    = 66.77
WHEEL_BASE_WIDTH  = 106.14
ACCURACY_SPEED      = 150    # calibrated speed for accurate distance and spin turns
ENCODER_TICKS_PER_ROTATION = 16  # MAGIC NUMBER TO MAKE NEW GOPIGO3 KIT WORK

def setParameters(egpg, wd=WHEEL_DIAMETER, wbw=WHEEL_BASE_WIDTH, spd=ACCURACY_SPEED, etpr=ENCODER_TICKS_PER_ROTATION, verbose=False):
    egpg.WHEEL_DIAMETER = wd
    egpg.WHEEL_CIRCUMFERENCE = wd * pi
    egpg.WHEEL_BASE_WIDTH = wbw
    egpg.WHEEL_BASE_CIRCUMFERENCE = wbw * pi
    egpg.DEFAULT_SPEED = ACCURACY_SPEED
    egpg.set_speed(spd)
    egpg.ENCODER_TICKS_PER_ROTATION = etpr  # Old GoPiGo3 = 6, New GoPiGo3 = 16
    egpg.MOTOR_TICKS_PER_DEGREE = ((egpg.MOTOR_GEAR_RATIO * egpg.ENCODER_TICKS_PER_ROTATION) / 360.0) # encoder ticks per output shaft rotation degree

    if verbose:
        print("egpg.WHEEL_DIAMETER   set to {}".format(egpg.WHEEL_DIAMETER))
        print("egpg.WHEEL_BASE_WIDTH set to {}".format(egpg.WHEEL_BASE_WIDTH))
        print("egpg.DEFAULT_SPEED set to {} DPS".format(egpg.DEFAULT_SPEED))
        print("egpg.speed set to {}".format(egpg.speed))
        print("egpg.ENCODER_TICKS_PER_ROTATION set to {}".format(egpg.ENCODER_TICKS_PER_ROTATION))
        print("egpg.MOTOR_TICKS_PER_DEGREE set to {}".format(egpg.MOTOR_TICKS_PER_DEGREE))


def main():
    import easygopigo3
    import time

    egpg = easygopigo3.EasyGoPiGo3(use_mutex=True)
    setParameters(egpg,verbose=True)
    time.sleep(5)

if __name__ == "__main__":  main()
