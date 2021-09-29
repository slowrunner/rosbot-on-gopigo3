#!/bin/bash
#
# stat_log1k.sh    print total travel, rotation, travel time and segments of travel in log1k.log
#
# requires bc  (sudo apt-get install bc)
#
# Note: bc scale=x only works for division!
#
FILE="log1k.log"
echo "1K STATS ON "$FILE

echo " "
totalTravel=`(awk -F'travel:' '{sum+=sqrt($2^2)}END{printf "%.1f", sum/1000;}' $FILE)`
totalTravelFt=`(echo "scale=1; ($totalTravel / 0.3048)" | bc)`
echo "Total Travel: " $totalTravel "m" $totalTravelFt "ft"
totalRotate=`(awk -F'rotation:' '{sum+=sqrt($2^2)}END{printf "%.1f", sum;}' $FILE)`
totalRevs=`(echo "scale=1; $totalRotate / 360" | bc)`
echo "Total Rotate: " $totalRotate "deg" $totalRevs "revolutions"
totalMotion=`(awk -F'motion:' '{sum+=$2}END{printf "%.1f", sum;}' $FILE)`
totalMotionHrs=`(echo "scale=3; $totalMotion / 3600" | bc)`
echo "Total Motion: " $totalMotion "sec" $totalMotionHrs "hrs"
aveSpeed=`(echo "scale=3; $totalTravel / $totalMotion" | bc)`
echo "Average Speed: " $aveSpeed "m/sec"
