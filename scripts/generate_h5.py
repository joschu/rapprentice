#!/usr/bin/env python

"""
Generate hdf5 file based on a yaml task file that specifies bag files and annotation files
"""

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("task_file")
parser.add_argument("--no_prompt", action="store_true")
parser.add_argument("--cloud_proc_func", default="extract_red")
parser.add_argument("--cloud_proc_mod", default="rapprentice.cloud_proc_funcs")
parser.add_argument("--no_clouds")
parser.add_argument("--clouds_only", action="store_true")
args = parser.parse_args()



import os, os.path as osp
import rosbag
import h5py
from rapprentice import bag_proc
import yaml
import importlib, inspect
import numpy as np

cloud_proc_mod = importlib.import_module(args.cloud_proc_mod)
cloud_proc_func = getattr(cloud_proc_mod, args.cloud_proc_func)


task_dir = osp.dirname(args.task_file)

with open(args.task_file, "r") as fh: task_info = yaml.load(fh)
h5path = osp.join(task_dir, task_info["h5path"].strip())


if args.clouds_only:
    hdf = h5py.File(h5path, "r+")
else:
    if osp.exists(h5path):    
        os.unlink(h5path)
    hdf = h5py.File(h5path)

    
    bag_infos = task_info["bags"]

    for (i_bag, bag_info) in enumerate(bag_infos):
        bag_file = osp.join(task_dir, bag_info["bag_file"])
        ann_file = osp.join(task_dir, bag_info["annotation_file"])
        video_dir = bag_info["video_dir"]

        demo_name = bag_info["demo_name"] if "demo_name" in bag_info else "demo%i"%i_bag
    
        bag = rosbag.Bag(bag_file)
        with open(ann_file, "r") as fh: annotations = yaml.load(fh)
    
        bag_proc.add_bag_to_hdf(bag, annotations, hdf, demo_name)
        bag_proc.add_rgbd_to_hdf(osp.join(task_dir, video_dir), annotations, hdf, demo_name)
    
    

### Now process all of the point clouds ####
if not args.no_clouds:
    for (seg_name, seg_info) in hdf.items():
        
        for field in ["cloud_xyz", "cloud_proc_func", "cloud_proc_mod", "cloud_proc_code"]:
            if field in seg_info: del seg_info[field]
        
        seg_info["cloud_xyz"] = cloud_proc_func(np.asarray(seg_info["rgb"]), np.asarray(seg_info["depth"]), np.asarray(seg_info["T_w_k"]))
        seg_info["cloud_proc_func"] = args.cloud_proc_func
        seg_info["cloud_proc_mod"] = args.cloud_proc_mod
        seg_info["cloud_proc_code"] = inspect.getsource(cloud_proc_func)
        
        
