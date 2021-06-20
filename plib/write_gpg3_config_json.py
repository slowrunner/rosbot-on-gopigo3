#!/usr/bin/env python3

# FILE: write_gpg3_config_json.py

# PURPOSE:  save robot constants to /home/pi/Dexter/gpg3_config.json
"""
Default GoPiGo3 file contents

.. code-block:: json
        
            {
                "wheel-diameter": 66.5,
                "wheel-base-width": 117,
                "ticks": 6,
                "motor_gear_ratio": 120
            }

"""
import json
import sys
sys.path.insert(1,"/home/pi/rosbot-on-gopigo3/plib/")
import easygopigo3
import myconfig


def write_robot_constants_file(config_file_path="/home/pi/Dexter/gpg3_config.json"):
    egpg = easygopigo3.EasyGoPiGo3(use_mutex = True)
    myconfig.setParameters(egpg)
    # egpg.save_robot_constants()
    with open(config_file_path, 'w') as json_file:
        data = {
                "wheel-diameter": egpg.WHEEL_DIAMETER,
                "wheel-base-width": egpg.WHEEL_BASE_WIDTH,
                "ticks": egpg.ENCODER_TICKS_PER_ROTATION,
                "motor_gear_ratio": egpg.MOTOR_GEAR_RATIO
        }
        json.dump(data, json_file)

    print("saved robot constants to {}".format(config_file_path))


def main():
    write_robot_constants_file()

if __name__ == "__main__": main()
