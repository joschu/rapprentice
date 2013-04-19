#!/usr/bin/env python

"""
Generate hdf5 file based on a yaml task file that specifies bag files and annotation files
"""

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("task_file")
parser.add_argument("--no_prompt", action="store_true")
args = parser.parse_args()



import os, os.path as osp
import rosbag
import h5py
from rapprentice.yes_or_no import yes_or_no
from rapprentice import bag_proc
import yaml

with open(args.task_file, "r") as fh: task_info = yaml.load(fh)


task_name = task_info["name"].strip()

task_dir = osp.dirname(args.task_file)


h5path = osp.join(task_dir, task_info["h5path"].strip())
if osp.exists(h5path):
    
    if args.no_prompt or yes_or_no("delete old file? %s"%h5path):
        os.unlink(h5path)
        
    
hdf = h5py.File(h5path)

    
bag_infos = task_info["bags"]

for bag_info in bag_infos:
    bag_file = osp.join(task_dir, bag_info["bag_file"])
    ann_file = osp.join(task_dir, bag_info["annotation_file"])
    video_dir = bag_info["video_dir"]
    
    bag = rosbag.Bag(bag_file)
    with open(ann_file, "r") as fh: annotations = yaml.load(fh)
    
    bag_proc.add_bag_to_hdf(bag, annotations, hdf)
    bag_proc.add_rgbd_to_hdf(osp.join(task_dir, video_dir), annotations, hdf)
    
    

    