#!/usr/bin/env python3

# FILE: print_gpg3_config_json.py

# PURPOSE:  print robot constants stored in /home/pi/Dexter/gpg3_config.json
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


def print_robot_constants_file(config_file_path="/home/pi/Dexter/gpg3_config.json"):

    with open(config_file_path, 'r') as json_file:
        data = json.load(json_file)

    print("CONTENT of {}".format(config_file_path))
    print(data)
    print("\nVALUES:")

    if 'wheel-diameter' in data:
        print('"wheel-diameter" : {}'.format(data['wheel-diameter']))
    if 'wheel-base-width' in data:
        print('"wheel-base-width" : {}'.format(data['wheel-base-width']))
    if 'ticks' in data:
        print('"ticks" : {}'.format(data['ticks']))
    if 'motor_gear_ratio' in data:
        print('"motor_gear_ratio" : {}'.format(data['motor_gear_ratio']))




if __name__ == "__main__": print_robot_constants_file()
