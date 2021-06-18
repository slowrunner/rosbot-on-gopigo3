#!/bin/bash

echo "Routine Shutdown Requested"
batt=`(/home/pi/rosbot-on-gopigo3/plib/battery.py)`
/home/pi/rosbot-on-gopigo3/logMaintenance.py "Routine Shutdown"
/home/pi/rosbot-on-gopigo3/logMaintenance.py "'$batt'"
sudo shutdown -h +2
