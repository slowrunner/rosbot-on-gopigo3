#!/usr/bin/env python3
#
# FILE:  egpg_center.py
# Results:  When you run this program, the ROSbot Servo will face center, and turn off.

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import easygopigo3 # import the GoPiGo3 drivers

egpg = easygopigo3.EasyGoPiGo3(use_mutex=True) # Create an instance of the GoPiGo3 class. GPG will be the GoPiGo3 object.
ps = egpg.init_servo()

# SERVO_1_CENTER = 1424
SERVO_1_CENTER_DEG = 85
# SERVO_1_LEFT = 2098    # +674  70 degrees left of center
# SERVO_1_RIGHT = 750    # -674  70 degrees right of center
# SERVO_OFF = 0

try:
    # GPG.set_servo(GPG.SERVO_1, SERVO_1_CENTER)
    ps.rotate_servo(SERVO_1_CENTER_DEG)    # (from easysensors)
    time.sleep(1)

except KeyboardInterrupt:
    ps.rotate_servo(SERVO_1, SERVO_1_CENTER_DEG)
    time.sleep(1)
finally:
    ps.disable_servo()        # relax servo (from easysensors)
