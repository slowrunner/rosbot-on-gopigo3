#!/usr/bin/env python3
#
# https://www.dexterindustries.com/GoPiGo/
# https://github.com/DexterInd/GoPiGo3
#
# Copyright (c) 2017 Dexter Industries
# Released under the MIT license
# (http://choosealicense.com/licenses/mit/).
#
# For more information see
# https://github.com/DexterInd/GoPiGo3/blob/master/LICENSE.md
#
# This code is an example for controlling the GoPiGo3 motors.  This uses
# the EasyGoPiGo3 library.  You can find more information on the library
# here:  http://gopigo3.readthedocs.io/en/latest/api-basic.html#easygopigo3
#
# Results:  The GoPiGo3 will move forward for 2 seconds, and then
# backward for 2 second.


# import the time library for the sleep function
import time
# import the GoPiGo3 drivers
from easygopigo3 import EasyGoPiGo3

# Create an instance of the GoPiGo3 class.
# GPG will be the GoPiGo3 object.
gpg = EasyGoPiGo3()


print("Move the motors forward freely for 1 second.")
gpg.forward()
time.sleep(1)
gpg.stop()

print("Stop the motors for 1 second.")
time.sleep(1)

print("Move the motors backward freely for 1 second.")
gpg.backward()
time.sleep(1)
gpg.stop()

print("Stop the motors for 5 seconds.")
time.sleep(5)

print("Drive the motors 50 cm and then stop.")
gpg.drive_cm(50, True)
time.sleep(1)

print("Drive the motors backward 50 cm and then stop.")
gpg.drive_cm(-50, True)
time.sleep(1)

print("Turn right 1 second.")
gpg.right()
time.sleep(1)

print("Turn left 1 second.")
gpg.left()
time.sleep(1)

print("Spin right 1 second.")
gpg.spin_right()
time.sleep(1)

print("Spin left 1 second.")
gpg.spin_left()
time.sleep(1)

print("Turn 90 degrees right")
gpg.turn_degrees(90)
time.sleep(1)

print("Turn 180 degrees left")
gpg.turn_degrees(-180)
time.sleep(1)

print("Turn 90 degrees right")
gpg.turn_degrees(90)
time.sleep(1)


print("Stop!")
gpg.stop()
print("Done!")
