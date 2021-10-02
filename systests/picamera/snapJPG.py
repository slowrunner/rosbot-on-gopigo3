#!/usr/bin/env python3

# FILE: snapJPG.py

# Since PiCamera is not available for 64-bit systems, 
# Uses raspistill to take single jpeg photos, or time lapse photos at 1 per second
#
# Time Lapse frame rate greater than 1 per second will drop frames

"""
Usage: snapJPG.py [-h] [-tl TIMELAPSE]

Uses raspistill to take a single jpg image, or time lapse images at 1 fps.

optional arguments:
  -h, --help            show this help message and exit
  -tl TIMELAPSE, --timelapse TIMELAPSE
                        Snap timelapse at 1fps for TIMELAPSE seconds

Output to ./images/rstill_YYYY-MM-DD_HH-MM-SS [_<timelapse image number>] .jpg
"""

from time import sleep
from datetime import datetime
import os
import argparse

# AVAILABLE RESOLUTIONS
RES_640x480 = (640, 480)         # 4:3 Full FOV
RES_1296x972 = (1296, 972)       # 4:3 Full FOV
RES_1296x730 = (1296, 730)       # 16:9 Partial Vertical FOV ?
RES_1920x1080 = (1920, 1080)     # 16:9 Center Partial H and V FOV
RES_2592x1944 = (2592, 1944)     # 4:3 Full FOV

# RESOLUTION TO USE FOR snapJPG.py
RES = RES_640x480

# default filename root
FN = "images/rstill_"
TL_DEFAULT_FPS = 1   # one frame per second


def snapImage(fn=FN, wh=RES, dur_s=0, fps=TL_DEFAULT_FPS):
    # print("snapImage(fn={},  wh={},  dur_s={},  fps={})".format(fn, wh, dur_s, fps))
    width = wh[0]
    height = wh[1]
    if dur_s > 0:
        period_ms = int(1000/fps)
        dur_ms = int((dur_s-1) * 1000)    # take off to prevent extra photo
        print("Taking {} by {} time lapse images for {} s at {} fps".format(width,height,dur_s, fps))
        fn=fn+datetime.now().strftime("%Y%m%d-%H%M%S")+"_%04d.jpg"
        cmd ='raspistill -n -rot 180 -w {} -h {} -sh 75 -t {} -tl {} -o {}'.format(width,height,dur_ms,period_ms,fn)
        print("cmd: {}".format(cmd))
        os.system(cmd)
        print("Wrote {}  ({} by {}) images as {}".format(int(dur_s*fps),width, height, fn))

    else:
        print("Taking single image")
        fn=fn+datetime.now().strftime("%Y%m%d-%H%M%S")+".jpg"
        cmd ='raspistill -n -rot 180 -w {} -h {} -sh 75 -o {}'.format(width,height,fn)
        print("cmd: {}".format(cmd))
        os.system(cmd)
        print("Wrote single {} by {} image to {}".format(width, height, fn))

def main():

    # ARGUMENT PARSER
    ap = argparse.ArgumentParser(
             description="Uses raspistill to take a single jpg image, or time lapse images at 1 fps.",
             epilog="Output to ./images/rstill_YYYY-MM-DD_HH-MM-SS [_<timelapse image number>] .jpg")
    ap.add_argument("-tl", "--timelapse", type=int, default=0,
                    help="Snap timelapse at 1fps for TIMELAPSE seconds")
    args = vars(ap.parse_args())
    tl_duration_s = args['timelapse']

    try:

        if not os.path.exists('images'):
            os.makedirs('images')

        if (tl_duration_s > 0):
            snapImage(dur_s=tl_duration_s)
        else:
            snapImage()

    except KeyboardInterrupt:
        print("\nCtrl-C Detected, exiting...")
    except Exception as e:
        print("Exception: {}".format(str(e)))


if __name__ == "__main__":
    main()
