#!/usr/bin/python3
#
# safetyShutdown.py     SAFELY SHUTDOWN AT BATTERY:9.15v READING:8.5v
#
#      Loop reading the battery voltage
#        UNTIL voltage stays below LOW_BATTERY_V 4 times,
#        then will force a shutdown.
#
#      Will start wifi led blinking orange 0.25v before safety shutdown voltage
#
#      Note: actual battery voltage is 0.65v higher than reading
#            due to reverse polarity protection diode
#

import sys
import time
import signal
import os
from datetime import datetime

sys.path.append("/home/pi/rosbot-on-gopigo3/plib")
import easygopigo3
import myconfig
import leds


LOW_BATTERY_V = 9.15   # (9.15-0.65 = 3cells x 3.05v ~7% reserve)
# LOW_BATTERY_V = 12.0   # TEST TEST TEST
REV_PROTECT_DIODE_DROP = 0.65
LOW_READING_V = LOW_BATTERY_V - REV_PROTECT_DIODE_DROP
WARNING_LED_V = LOW_READING_V + 0.25

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

  print("\n********* ROSbot safetyShutdown STATUS *****")
  print(datetime.now().date(), getUptime())
  vBatt = egpg.volt()  #egpg.get_voltage_battery()
  print("Battery Voltage: %0.2f" % vBatt)
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
  print("safetyShutdown.py: handle_ctlc() executed")

def main():
  global egpg

  # #### SET CNTL-C HANDLER 
  set_cntl_c_handler(handle_ctlc)

  # #### Create instance of GoPiGo3 base class 
  egpg = easygopigo3.EasyGoPiGo3(use_mutex=True)
  myconfig.setParameters(egpg,verbose=True)

  batteryLowCount = 0
  warning_led_on = False

  try:
    while True:
        printStatus()
        vBatt = egpg.volt()
        if (vBatt < LOW_READING_V):
            batteryLowCount += 1
        else: batteryLowCount = 0
        if (warning_led_on == False) and (vBatt < WARNING_LED_V):
            warning_led_on = True
            leds.wifi_blinker_on(egpg,color=leds.ORANGE)
        if (batteryLowCount > 3):
          print ("WARNING, WARNING, SHUTTING DOWN NOW")
          print ("BATTERY %.2f volts BATTERY LOW - SHUTTING DOWN NOW" % vBatt)
          egpg.reset_all()
          time.sleep(1)
          os.system("/home/pi/rosbot-on-gopigo3/logMaintenance.py 'SAFETY SHUTDOWN - BATTERY LOW'")
          time.sleep(1)
          # os.system("sudo shutdown +10")   # for testing
          os.system("sudo shutdown -h now")
          sys.exit(0)
        time.sleep(10)    # check battery status every 10 seconds
                          # important to make four checks low V quickly
    #end while
  except SystemExit:
    print("safetyShutdown.py: exiting")

if __name__ == "__main__":
    main()



