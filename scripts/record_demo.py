#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("prefix")
parser.add_argument("--downsample", default=3, type=int)
args = parser.parse_args()

import subprocess, signal
from rapprentice.colorize import colorize
import time

started_bag = False
started_video = False

localtime   = time.localtime()
time_string  = time.strftime("%Y-%m-%d-%H-%M-%S", localtime)

basename = args.prefix + "_" + time_string

subprocess.call("killall XnSensorServer", shell=True)

try:

    bag_cmd = "rosbag record /joint_states /joy -O %s"%basename
    print colorize(bag_cmd, "green")
    bag_handle = subprocess.Popen(bag_cmd, shell=True)
    started_bag = True
    
    video_cmd = "record_rgbd_video --out=%s --downsample=%i"%(basename, args.downsample)
    print colorize(video_cmd, "green")
    video_handle = subprocess.Popen(video_cmd, shell=True)
    started_video = True
    
    time.sleep(9999)    
except KeyboardInterrupt:
    print colorize("got control-c", "green")

finally:
    
    if started_bag:
        bag_handle.send_signal(signal.SIGINT)
        bag_handle.wait()
    if started_video:
        video_handle.send_signal(signal.SIGINT)
        video_handle.wait()
    
