#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("h5file")
parser.add_argument("--cloud_proc_func", default="extract_red")
parser.add_argument("--cloud_proc_mod", default="rapprentice.cloud_proc_funcs")

args = parser.parse_args()

import h5py
import importlib, inspect
import numpy as np

cloud_proc_mod = importlib.import_module(args.cloud_proc_mod)
cloud_proc_func = getattr(cloud_proc_mod, args.cloud_proc_func)



hdf = h5py.File(args.h5file)


for (seg_name, seg_info) in hdf.items():
    
    for field in ["cloud_xyz", "cloud_proc_func", "cloud_proc_mod", "cloud_proc_code"]:
        if field in seg_info: del seg_info[field]
    
    seg_info["cloud_xyz"] = cloud_proc_func(np.asarray(seg_info["rgb"]), np.asarray(seg_info["depth"]), np.asarray(seg_info["T_w_k"]))
    seg_info["cloud_proc_func"] = args.cloud_proc_func
    seg_info["cloud_proc_mod"] = args.cloud_proc_mod
    seg_info["cloud_proc_code"] = inspect.getsource(cloud_proc_func)
    