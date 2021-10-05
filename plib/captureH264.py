#!/usr/bin/env python3

# FILE: captureH264.py

# Since PiCamera is not available for 64-bit systems,
# Uses raspivid to take h264 video through the pi camera

"""
usage: captureH264.py [-h] [-t TIME] [-fps FRAMERATE] [-nr NAMEROOT]

Uses raspivid to capture video in h264 container

optional arguments:
  -h, --help            show this help message and exit
  -t TIME, --time TIME  Capture video for TIME seconds
  -fps FRAMERATE, --framerate FRAMERATE
                        Frames Per Second - Default 1.0 fps
  -nr NAMEROOT, --nameroot NAMEROOT
                        name root - Default "videos/capture_"

Output to <nameroot>_YYYY-MM-DD_HH-MM-SS.h264
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
FN = "videos/capture_"
DEFAULT_FPS = 1


def takeVideo(fnr=FN, wh=RES, dur_s=5, fps=DEFAULT_FPS):
    width = wh[0]
    height = wh[1]
    dur_ms = int(dur_s * 1000)
    print("Starting {} by {} video capture for {} s at {} fps".format(width,height,dur_s, fps))
    fn=fnr+datetime.now().strftime("%Y%m%d-%H%M%S")+".h264"
    cmd ='raspivid -n -rot 180 -w {} -h {} -sh 75 -t {} -fps {} -o {}'.format(width,height,dur_ms,fps,fn)
    print("cmd: {}".format(cmd))
    os.system(cmd)
    print("Wrote ({} by {}) video as {}".format(width, height, fn))
    return fn


def main():

    # ARGUMENT PARSER
    ap = argparse.ArgumentParser(
             description="Uses raspivid to capture video in h264 container",
             epilog="Output to <nameroot>_YYYY-MM-DD_HH-MM-SS.h264")
    ap.add_argument("-t", "--time", type=int, default=5,
                    help="Capture video for TIME seconds")
    ap.add_argument("-fps", "--framerate", type=float, default=1.0,
                    help="Frames Per Second - Default 1.0 fps")
    ap.add_argument("-nr", "--nameroot", default=FN,
                    help='name root - Default "videos/capture_"')
    args = vars(ap.parse_args())
    t_duration_s = int(args['time'])
    framerate = float(args['framerate'])
    fnroot = args['nameroot']
    outfile = ""

    try:

        if not os.path.exists('videos'):
            os.makedirs('videos')

        outfile = takeVideo(dur_s=t_duration_s, fps=framerate, fnr=fnroot)

    except KeyboardInterrupt:
        print("\nCtrl-C Detected, exiting...")
    except Exception as e:
        print("Exception: {}".format(str(e)))


if __name__ == "__main__":
    main()
