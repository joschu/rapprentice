#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("bagfile")
parser.add_argument("outfile")
args = parser.parse_args()
from rapprentice import bag_proc

import yaml, rosbag

bag = rosbag.Bag(args.bagfile)
stamps, meanings = bag_proc.extract_joy(bag)
print "joystick button presses:"
for (stamp, meaning) in zip(stamps, meanings):
    print "\t%.4f: %s"%(stamp/1.e9, meaning)

seg_infos = bag_proc.joy_to_annotations(stamps, meanings)
for (i_seg, seg_info) in enumerate(seg_infos): 
    seg_info["name"] = "seg%.2i"%i_seg
    seg_info["description"] = "(no description)"
print "segment info: "
print seg_infos



print "writing to %s"%args.outfile
with open(args.outfile,"w") as fh:
    yaml.dump(seg_infos,fh)
print "done"
    
