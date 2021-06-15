#!/bin/bash

echo "Routine Shutdown Requested"
/home/pi/rosbot-on-gopigo3/logMaintenance.py "Routine Shutdown"
sudo shutdown -h +10
