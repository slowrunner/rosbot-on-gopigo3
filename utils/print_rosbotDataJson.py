#!/usr/bin/env  python3

import sys
sys.path.insert(1,"/home/pi/rosbot-on-gopigo3/plib")

import rosbotDataJson

# print("carlDataJson contents:")
# lcarlData = carlDataJson.getCarlData()
# for i in lcarlData:
#     print("  ",i," : ",lcarlData[i])

rosbotDataJson.printData()
