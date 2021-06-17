#!/usr/bin/env python3

# file:  rosbotDataJson.py
#
# Serialize data values to /home/pi/rosbot-on-gopigo3/rosbotData.json
#
# Methods:
#    saveData(dataname, datavalue, logit=False)   # adds datanaem:datavalue to rosbotData.json file
#    getData(dataname=None)      # either returns dictionary with all values, or just value of passed name
#    delData(dataname)           # delete item from rosbotData.json
#    printData()                 # prints contents of rosbotData.json
#
import sys
sys.path.append('/home/pi/rosbot-on-gopigo3/plib')

import json
import threading
import runLog

DataLock = threading.Lock()       # with DataLock: any operation to make syncronous

def saveData(dataname, datavalue, logit=False):


    # print("-- saveData({},{}) called".format(dataname, datavalue))
    with DataLock:         # prevents two different saveData() at same time
        lData = {}
        # print("got lock")
        try:

            lData = getData()   # lock assures no one changes this till we are done
            if lData == None:
                lData = {}
            # print("   Data:", lData)
            lData[dataname] = datavalue
            # print("   lData:",lData)

            with open('/home/pi/rosbot-on-gopigo3/rosbotData.json', 'w') as outfile:
                json.dump( lData, outfile )
            # print("   Data.json updated")
            if logit: runLog.logger.info("** Data '{}' = {} updated **".format(dataname, datavalue))
        except:
            # print("   saveData failed")
            return False

        return True

def delData(dataname):
    # print("-- delData({}) called".format(dataname))

    with DataLock:
        lData = {}
        try:

            lData = getData()
            if lData == None:
               lData = ()
            # print("   Data:", lData)
            if dataname in lData: 
                del lData[dataname]
                # print("   lData:", lData)

                with open('/home/pi/rosbot-on-gopigo3/rosbotData.json', 'w') as outfile:
                    json.dump( lData, outfile )
                # print("   Data.json updated")
            # else:   print("   {} not found in Data".format(dataname))
        except:
            # print("   delData{} failed".dataname)
            return False

        return True


def getData(dataname=None):


    # print("-- getData({}) called".format(dataname))

    try:
        with open('/home/pi/rosbot-on-gopigo3/rosbotData.json', 'r') as infile:
            lData = json.load(infile)
            if (dataname == None):
                return lData
            else:
                return lData[dataname]
    except:
        # print("   getData() except")
        return None

def printData():
    print("rosbotData.json contents:")
    lData = getData()
    if lData != None:
        for i in lData:
            print("  ",i," : ",lData[i])
    else:
        print("rosbotData.json contains no data")

def main():

    print("** Starting main()")

    printData()

    lData = getData()
    print("    rosbotData: ",lData)

    chargeCycles = 1

    if (saveData('chargeCycles', chargeCycles) == True):
        print('   Saved chargeCycles: {}'.format(chargeCycles))
    else:
        print("   saveData('chargeCycles') failed")

    lData = getData()
    print("    rosbotData: ",lData)


    chargeCycles = int(getData('chargeCycles'))
    chargeCycles += 1

    if (saveData('chargeCycles', chargeCycles) == True):
        print('   Saved chargeCycles: {}'.format(chargeCycles))
    else:
        print("   saveData('chargeCycles') failed")


    if 'chargeCycles' in lData:
        print("removing chargeCycles from rosbotData.json")
        delData('chargeCycles')

    if (saveData('nothing',"not important" ) == True):
        print('   Saved nothing: {}'.format("not important"))
    else:
        print("   saveData('nothing') failed")

    lData = getData()
    print("   Data: ",lData)


if __name__ == "__main__": 
    main()

