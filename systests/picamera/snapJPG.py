#!/usr/bin/env python3

# FILE: snapJPG.py

# Since PiCamera is not available for 64-bit systems, 
# Uses raspistill to take jpeg photos
#
# Time Lapse frame rate greater than 1 per second will drop frames

from time import sleep
from datetime import datetime
import os

RES_640x480 = (640, 480)         # 4:3 Full FOV
RES_1296x972 = (1296, 972)       # 4:3 Full FOV
RES_1296x730 = (1296, 730)       # 16:9 Partial Vertical FOV ?
RES_1920x1080 = (1920, 1080)     # 16:9 Center Partial H and V FOV
RES_2592x1944 = (2592, 1944)     # 4:3 Full FOV

RES = RES_640x480
FN = "images/rstill_%04d.jpg"
T_DEFAULT_SECS = 60   #  duration of time lapse
TL_DEFAULT_FPS = 1   # one frame per second


def snapImage(fn=FN, wh=RES, dur_s=T_DEFAULT_SECS, fps=0):
    print("snapImage(fn={},  wh={},  dur_s={},  fps={})".format(fn, wh, dur_s, fps))
    width = wh[0]
    height = wh[1]
    if fps > 0:
        period_ms = int(1000/fps)
        dur_ms = int((dur_s-1) * 1000)    # take off to prevent extra photo
        print("Taking {} by {} time lapse images for {} s at {} fps".format(width,height,dur_s, fps))
        cmd ='raspistill -n -vf -w {} -h {} -sh 75 -t {} -tl {} -o {}'.format(width,height,dur_ms,period_ms,fn)
        print("cmd: {}".format(cmd))
        os.system(cmd)
        print("Wrote {}  ({} by {}) images as {}".format(int(dur_s*fps),width, height, fn))

    else:
        print("Taking single image")
        cmd ='raspistill -n -vf -w {} -h {} -sh 75 -o {}'.format(width,height,fn)
        print("cmd: {}".format(cmd))
        os.system(cmd)
        print("Wrote single {} by {} image to {}".format(width, height, fn))

def main():

    try:
        rate = 1
        res = RES
        dur = T_DEFAULT_SECS

        if not os.path.exists('images'):
            os.makedirs('images')

        if (rate > 0):
            fname = FN
            snapImage(fn=fname,wh=res,dur_s=dur,fps=rate)
        else:
            fname = "images/rstill_"+datetime.now().strftime("%Y%m%d-%H%M%S")+".jpg"
            snapImage(fn=fname,wh=res,dur_s=dur,fps=0)

    except KeyboardInterrupt:
        print("\nCtrl-C Detected, exiting...")
    except Exception as e:
        print("Exception: {}".format(str(e)))


if __name__ == "__main__":
    main()
