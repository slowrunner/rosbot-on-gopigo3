#!/usr/bin/env python3
#
# https://www.dexterindustries.com/GoPiGo/
# https://github.com/DexterInd/GoPiGo3
#
# Copyright (c) 2017 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information see https://github.com/DexterInd/GoPiGo3/blob/master/LICENSE.md
#
# This code is an example for controlling the GoPiGo3 Servos
#
# Results:  When you run this program, the GoPiGo3 Servos will rotate back and forth.

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import gopigo3 # import the GoPiGo3 drivers

GPG = gopigo3.GoPiGo3() # Create an instance of the GoPiGo3 class. GPG will be the GoPiGo3 object.

SERVO_1_CENTER = 1424
SERVO_1_LEFT = 2098    # +674  70 degrees left of center
SERVO_1_RIGHT = 750    # -674  70 degrees right of center
try:

    while True:
        # GPG.set_servo(GPG.SERVO_1, SERVO_1_CENTER)
        # time.sleep(1)
        GPG.set_servo(GPG.SERVO_1, SERVO_1_LEFT)
        time.sleep(1)
        GPG.set_servo(GPG.SERVO_1, SERVO_1_RIGHT)
        time.sleep(1)

        for i in range(SERVO_1_RIGHT, SERVO_1_LEFT):    # count from 1000 to 2000
            GPG.set_servo(GPG.SERVO_1, i)
            # print(i)
            time.sleep(0.001)
        time.sleep(1)

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    GPG.set_servo(GPG.SERVO_1, SERVO_1_CENTER)
    time.sleep(1)
    GPG.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the GoPiGo3 firmware.
