#!/usr/bin/env python3

# FILE: convert264toMP4.py

# This program converts h264 video,
# (previously captured using raspistill or captureH264.py)
# to mp4 container
#

"""
usage: convertH264toMP4.py [-h] [-fps FRAMERATE] -i INFILE

Uses ffmpeg to create mp4 copy of .h264 file

optional arguments:
  -h, --help            show this help message and exit
  -fps FRAMERATE, --framerate FRAMERATE
                        Frames Per Second - Default 1.0 fps
  -i INFILE, --infile INFILE
                        input filename"

Outputs: <infile>.mp4
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



def createMP4fromH264(infile,fps=DEFAULT_FPS):
    of=infile+".mp4"
    cmd ='ffmpeg -r {} -i {} -y -vcodec copy {}'.format(fps,infile,of)
    print("cmd: {}".format(cmd))
    os.system(cmd)
    print("Wrote {}".format(of))

def main():

    # ARGUMENT PARSER
    ap = argparse.ArgumentParser(
             description="Uses ffmpeg to create mp4 copy of .h264 file",

             epilog="Outputs: <infile>.mp4")
    ap.add_argument("-fps", "--framerate", type=float, default=1.0,
                    help="Frames Per Second - Default 1.0 fps")
    ap.add_argument("-i", "--infile", type=ascii, required=True,
                    help='input filename"')
    args = vars(ap.parse_args())
    framerate = float(args['framerate'])
    infn = args['infile']

    try:

        if not os.path.exists('videos'):
            os.makedirs('videos')

        if (infn != ""):
            createMP4fromH264(infile=infn,fps=framerate)

    except KeyboardInterrupt:
        print("\nCtrl-C Detected, exiting...")
    except Exception as e:
        print("Exception: {}".format(str(e)))


if __name__ == "__main__":
    main()
