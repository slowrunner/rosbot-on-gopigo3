#!/bin/bash

echo "Routine Reboot Requested"
batt=`(/home/pi/rosbot-on-gopigo3/plib/battery.py)`
/home/pi/rosbot-on-gopigo3/logMaintenance.py "Routine Reboot"
/home/pi/rosbot-on-gopigo3/logMaintenance.py "'$batt'"
sudo reboot
