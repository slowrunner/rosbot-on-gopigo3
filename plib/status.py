#!/usr/bin/python3

# status.py    Basic Status (thread-safe)


#      import status provides printStatus(egpg,ds)

#      ./status.py    will print status once and exit
#
#      ./status.py -l (or -loop) will print status every 5 seconds
#
#      ./status.py -d will print status without distance sensor
#
#      ./status.py -h (or --help) will print usage

"""
********* ROSbot STATUS *****
2021-08-05  08:33:37 up 2 min,  1 user,  load average: 1.14, 0.92, 0.38
Current Battery 12.05v EasyGoPiGo3 Reading 11.24v
5v Supply: 4.98
Processor Temp: 38.6'C
Clock Frequency: 1.30 GHz
throttled=0x0
Distance Sensor: nothing within 90 inches
"""


# IMPORTS
import sys
sys.path
sys.path.insert(1,'/home/pi/rosbot-on-gopigo3/plib')
import time
import signal
import os
import myPyLib
# import speak
import myconfig
from datetime import datetime
from noinit_easygopigo3 import EasyGoPiGo3
import battery
# import myDistSensor
import lifeLog
import runLog
import argparse
# from my_safe_inertial_measurement_unit import SafeIMUSensor
import rosbotDataJson as rosbotData

# Return CPU temperature as a character string
def getCPUtemperature():
    res = os.popen('vcgencmd measure_temp').readline()
    return(res.replace("temp=", "").replace("\n", ""))


# Return Clock Freq as a character string
def getClockFreq():
    res = os.popen('vcgencmd measure_clock arm').readline()
    res = int(res.split("=")[1])
    if (res < 1000000000):
        res = str(res/1000000)+" MHz"
    else:
        res = '{:.2f}'.format(res/1000000000.0)+" GHz"
    return res


# Return throttled flags as a character string
#   0x10001  under-voltage 4.63v occurred / occurring
#   0x20002  freq-cap occurred / occurring
#   0x40004  Temp Throttled occurred / occurring
#   0x80008  SOFT_TEMPERATURE_LIMIT (default 60degC, boot/config.txt temp_soft_limit=70 to increase)

def getThrottled():
    res = os.popen('vcgencmd get_throttled').readline()
    return res.replace("\n", "")


def getUptime():
    res = os.popen('uptime').readline()
    return res.replace("\n", "")

def getRoomTemp(imu):
    roomTemp = (imu.safe_read_temperature() * 9.0/5.0 + 32.0) - 0.7
    return roomTemp

def printStatus(egpg, ds):
    print("\n********* ROSbot STATUS *****")
    print("{} {}".format(datetime.now().date(), getUptime()))
    print(battery.voltages_string(egpg))
    if battery.on_last_leg(egpg):
        print("WARNING - Battery Is Nearing Shutdown Voltage")
    v5V = egpg.get_voltage_5v()
    print("5v Supply: %0.2f" % v5V)
    print("Processor Temp: %s" % getCPUtemperature())
    try:
        print("Estimated Room Temp: %.1F" % getRoomTemp(egpg.imu))
    except Exception:  #no imu defined by user of printStatus
        pass
    print("Clock Frequency: %s" % getClockFreq())
    print("%s" % getThrottled())
    if ds is not None:
        dist = ds.read_mm() / 25.4
        if dist < 90:
            print("Distance Sensor: %0.1f inches" % dist)
        else:
            print("Distance Sensor: nothing within 90 inches")


# ##### MAIN ######


def handle_ctlc():
    print("status.py: handle_ctlc() executed")


def main():
    # #### SET CNTL-C HANDLER
    myPyLib.set_cntl_c_handler(handle_ctlc)

    # #### Create a mutex protected instance of EasyGoPiGo3 base class
    egpg = EasyGoPiGo3(use_mutex=True,noinit=True)
    # myconfig.setParameters(egpg)
    # egpg.imu = SafeIMUSensor(port = "AD1", use_mutex = True, init = False)


    # ARGUMENT PARSER
    ap = argparse.ArgumentParser()
    ap.add_argument("-l", "--loop", default=False, action='store_true',
                    help="optional loop mode")
    ap.add_argument("-d", "--distance_sensor", default=True,
                    action='store_false', help="no distance sensor")

    args = vars(ap.parse_args())
    loopFlag = args['loop']
    dsFlag = args['distance_sensor']

    # ### Create (protected) instance of EasyDistanceSensor
    if dsFlag:
        ds = egpg.init_distance_sensor()
    else:
        ds = None

    if loopFlag:
        # runLog.logger.info(strStart)
        strStart = "Starting status.py - "+battery.voltages_string(egpg)
        runLog.entry(strStart)

    # print ("Starting status loop at %.2f volts" % battery.volts())
    try:
        while True:
            time.sleep(5)
            printStatus(egpg, ds)
            vBatt = egpg.volt()
            if (loopFlag is False):
                break
        # end while
    except SystemExit:
        strToLog = "Exiting status.py - "+battery.voltages_string(egpg)

        if loopFlag:
            # runLog.logger.info(strToLog)
            runLog.entry(strToLog)
        print(strToLog)
        time.sleep(1)


if __name__ == "__main__":
    main()
