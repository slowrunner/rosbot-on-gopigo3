#!/bin/bash

kill $(ps aux | grep '[p]ython3 snapJPG.py' | awk '{print $2}')
killall raspistill
