#!/usr/bin/python3
#
# FILE: tillshutdownLife.py     Battery Protection Cirucuit Cut-out Life Measurement
# 
# PURPOSE:     Run Battery down (from battery protection charge to full battery protection discharge
#              while printing status every 10 seconds to command line
#
# WARNING - WARNING Since the battery will yank the power without warning
#                   SD card corruption is possible (not probable, but possible)
#
#      Note: Actual battery voltages are 0.6v to 0.8v higher than GoPiGo3 reading
#            due to the voltage drop of the GoPiGo3's reverse polarity protection diode, wires and connectors


import sys
import time
import signal
import os
from datetime import datetime

import easygopigo3

REV_PROTECT_DIODE = 0.81    # The GoPiGo3 has a reverse polarity protection diode drop of 0.6v to 0.8v (n=2)
WARNING_LOW_vBatt = 10.0       # Give Advance Warning battery is around "the knee" (~20 minutes to safety shutdown vBatt )
SAFETY_SHUTDOWN_vBatt = 9.75   # Battery Discharge Protection Circuit allows down to 8.15v (~15 minutes reserve at 9.75v)
IGNORE_TOO_LOW = True          # Set False to test shutdown at SAFETY_SHUTDOWN_vBatt instead of running till battery fully discharged

# Return (approx) battery voltage and the actual GoPiGo3 voltage reading
def vBatt_vReading(egpg):
	vReading = egpg.volt()
	vBatt = vReading + REV_PROTECT_DIODE
	return vBatt,vReading

# Return a formatted string with the approx battery voltage and the actual GoPiGo3 reading
def voltages_string(egpg):
        vBatt, vReading = vBatt_vReading(egpg)
        return "Current Battery {:.2f}v EasyGoPiGo3 Reading {:.2f}v".format(vBatt,vReading)

# Return True if battery is below "safe shutdown voltage"
def too_low(egpg):
	vBatt, _ = vBatt_vReading(egpg)
	return vBatt < SAFETY_SHUTDOWN_vBatt

# Return True if battery is operating past "the knee" in the discharge curve
def on_last_leg(egpg):
	vBatt, _ = vBatt_vReading(egpg)
	return vBatt < WARNING_LOW_vBatt

# Return CPU temperature as a character string
def getCPUtemperature():
    res = os.popen('vcgencmd measure_temp').readline()
    return(res.replace("temp=","").replace("\n",""))

# Return Clock Freq as a character string
def getClockFreq():
    res = os.popen('vcgencmd measure_clock arm').readline()
    res = int(res.split("=")[1])
    if (res < 1000000000):
        res = str(res/1000000)+" MHz"
    else: res = '{:.2f}'.format(res/1000000000.0)+" GHz"
    return res

# Return throttled flags as a character string
def getThrottled():
    res = os.popen('vcgencmd get_throttled').readline()
    return res.replace("\n","")

def getUptime():
    res = os.popen('uptime').readline()
    return res.replace("\n","")


def printStatus():
  global egpg

  print("\n********* ROSbot tillShutdownLife.py STATUS *****")
  print(datetime.now().date(), getUptime())
  print(voltages_string(egpg))
  if too_low(egpg):
     print("WARNING - BATTERY IS TOO LOW")
  elif on_last_leg(egpg):
         print("WARNING - Battery Is Nearing Shutdown Voltage")
  v5V = egpg.get_voltage_5v()
  print("5v Supply: %0.2f" % v5V)
  print("Processor Temp: %s" % getCPUtemperature())
  print("Clock Frequency: %s" % getClockFreq())
  print("%s" % getThrottled())

# ######### CNTL-C #####
# Callback and setup to catch control-C and quit program

_funcToRun=None

def signal_handler(signal, frame):
  print('\n** Control-C Detected')
  if (_funcToRun != None):
     _funcToRun()
  sys.exit(0)     # raise SystemExit exception

# Setup the callback to catch control-C
def set_cntl_c_handler(toRun=None):
  global _funcToRun
  _funcToRun = toRun
  signal.signal(signal.SIGINT, signal_handler)




# ##### MAIN ######

def handle_ctlc():
  global egpg
  egpg.reset_all()
  print("status.py: handle_ctlc() executed")

def main():
  global egpg

  # #### SET CNTL-C HANDLER 
  set_cntl_c_handler(handle_ctlc)

  # #### Create instance of GoPiGo3 base class 
  egpg = easygopigo3.EasyGoPiGo3(use_mutex=True)
  batteryLowCount = 0

  #print ("Starting status loop at %.2f volts" % battery.volts())  
  try:
    while True:
        printStatus()
        if ((IGNORE_TOO_LOW != True) and too_low(egpg)):
            batteryLowCount += 1
        else: batteryLowCount = 0
        if (batteryLowCount > 3):
          vBatt,_ = vBatt_vReading(egpg)
          print ("WARNING, WARNING, SHUTTING DOWN NOW")
          print ("BATTERY %.2f volts BATTERY LOW - SHUTTING DOWN NOW" % vBatt)
          egpg.reset_all()
          time.sleep(1)
          os.system("sudo shutdown -h +2")
          sys.exit(0)
        time.sleep(10)    # check battery status every 10 seconds
                          # important to make four checks low V quickly      
    #end while
  except SystemExit:
    print("status.py: exiting")

if __name__ == "__main__":
    main()



