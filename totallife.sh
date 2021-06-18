#!/bin/bash
#
# totallife.sh    print total hours and sessions of life in life.log
#
# requires bc  (sudo apt-get install bc)
#
echo "TOTAL LIFE STATISTICS"
echo "(Cleaning life.log first)"
/home/pi/rosbot-on-gopigo3/cleanlifelog.py
echo " "
fn="/home/pi/rosbot-on-gopigo3/life.log"
totalLife=`(awk -F':' '{sum+=$3}END{print sum;}' $fn)`
echo "Total Life: " $totalLife "hrs (since June 10, 2021)"
echo "Sessions (boot): " `(grep -c "\- boot \-" $fn)`
booted=`(grep -c "\- boot \-" $fn)`
aveSession=`(echo "scale=1; ($totalLife / $booted)" | bc -l)`
echo "Average Session: " $aveSession "hrs"
safetyShutdowns=`(grep -c "SAFETY SHUTDOWN" $fn)`
echo "Safety Shutdowns: " $safetyShutdowns 
