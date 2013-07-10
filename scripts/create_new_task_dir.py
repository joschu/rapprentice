#!/usr/bin/env python
import argparse
parser = argparse.ArgumentParser(description="create default directory structure and files for a new task")
parser.add_argument("path", help='parent directory, e.g. "~/Data"')
parser.add_argument("taskname", help='description of task, e.g. "overhand"')
args = parser.parse_args()

import os,os.path as osp
datadir = osp.join(args.path, args.taskname)
assert not osp.exists(datadir)


os.mkdir(datadir)
with open(osp.join(datadir, args.taskname + ".yaml"),"w") as fh:
    fh.write(
"""
name: %s
h5path: %s
bags:
"""%(args.taskname, args.taskname+".h5"))
