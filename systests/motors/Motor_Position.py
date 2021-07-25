#!/usr/bin/env python3
#
# https://www.dexterindustries.com/GoPiGo/
# https://github.com/DexterInd/GoPiGo3
#
# Copyright (c) 2017 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information see https://github.com/DexterInd/GoPiGo3/blob/master/LICENSE.md
#
# This code is an example for controlling the GoPiGo3 Motors
#
# Results:  When you run this program, the GoPiGo3 Motors will rotate back and forth.


import time     # import the time library for the sleep function
import gopigo3 # import the GoPiGo3 drivers

GPG = gopigo3.GoPiGo3() # Create an instance of the GoPiGo3 class. GPG will be the GoPiGo3 object.

try:
    GPG.set_motor_limits(GPG.MOTOR_LEFT+GPG.MOTOR_RIGHT,power=50,dps=150)
    print('Motor_Position: encoders: ({},{})'.format(GPG.get_motor_encoder(GPG.MOTOR_LEFT),GPG.get_motor_encoder(GPG.MOTOR_RIGHT)))
    # Both encoder reset methods appear to work equally well
    GPG.reset_motor_encoder(GPG.MOTOR_LEFT + GPG.MOTOR_RIGHT)
    # GPG.offset_motor_encoder(GPG.MOTOR_LEFT, GPG.get_motor_encoder(GPG.MOTOR_LEFT))
    # GPG.offset_motor_encoder(GPG.MOTOR_RIGHT, GPG.get_motor_encoder(GPG.MOTOR_RIGHT))
    print('Motor_Position: encoders: ({},{})'.format(GPG.get_motor_encoder(GPG.MOTOR_LEFT),GPG.get_motor_encoder(GPG.MOTOR_RIGHT)))
    time.sleep(5)

    for i in range(0, 361):
        GPG.set_motor_position(GPG.MOTOR_LEFT + GPG.MOTOR_RIGHT, i)
        time.sleep(0.01)
        print('Motor_Position: encoders: ({},{})'.format(GPG.get_motor_encoder(GPG.MOTOR_LEFT),GPG.get_motor_encoder(GPG.MOTOR_RIGHT)))

    while True:
        for i in range(-360, 361):
            GPG.set_motor_position(GPG.MOTOR_LEFT + GPG.MOTOR_RIGHT, -i)
            time.sleep(0.01)
        print('Motor_Position: encoders: ({},{})'.format(GPG.get_motor_encoder(GPG.MOTOR_LEFT),GPG.get_motor_encoder(GPG.MOTOR_RIGHT)))

        for i in range(-360, 361):
            GPG.set_motor_position(GPG.MOTOR_LEFT + GPG.MOTOR_RIGHT, i)
            time.sleep(0.01)
        print('Motor_Position: encoders: ({},{})'.format(GPG.get_motor_encoder(GPG.MOTOR_LEFT),GPG.get_motor_encoder(GPG.MOTOR_RIGHT)))

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    GPG.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the GoPiGo3 firmware.
