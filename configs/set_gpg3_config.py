#!/usr/bin/env python3

# PURPOSE: Create /home/pi/Dexter/gpg3_config.json according to serial number list

from gopigo3 import GoPiGo3

gpg = GoPiGo3()
gpg.load_robot_constants()
