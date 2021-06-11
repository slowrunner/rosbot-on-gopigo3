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
declare -i newBattsAtCycle=0
# awk -F':' '{sum+=$3}END{print "total life: " sum " hrs";}' $fn
totalLife=`(awk -F':' '{sum+=$3}END{print sum;}' $fn)`
echo "Total Life: " $totalLife "hrs since June 10, 2021"
# echo "Sessions (boot) this year: " `(grep -c "\- boot \-" $fn)`
bootedThisYr=`(grep "\- boot \-" $fn | sort -u -k1,1 | wc -l)`
echo "Average Time Between Reboot: " $aveSession "hrs"
echo "New Batteries At Cycle:" $newBattsAtCycle
safetyShutdowns=`(grep -c "Safety Shutdown" $fn)`
echo "Safety Shutdowns this year: " $safetyShutdowns 
playtime=`(grep playtime $fn | awk -F"after" '{sum+=$2}END{print sum;}' )`
avePlaytime=`(echo "scale=1; $playtime / ($bootedThisYr)" | bc)`
echo "Ave Playtime this year: " $avePlaytime
