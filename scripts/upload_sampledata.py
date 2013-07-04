#!/usr/bin/env python

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--delete", action="store_true")
parser.add_argument("sampledata_dir")
args = parser.parse_args()

import subprocess,os
import os.path as osp
print "cding to %s"%osp.dirname(args.sampledata_dir)
os.chdir(osp.dirname(args.sampledata_dir))


print "creating tar file"
subprocess.check_call('tar cvf sampledata.tar sampledata',shell=True)
print "uploading"            
subprocess.check_call("rsync -azvu --progress %s sampledata pabbeel@rll.berkeley.edu:/var/www/rapprentice"%("--delete" if args.delete else ""), shell=True)
subprocess.check_call("rsync -azvu --progress %s sampledata.tar pabbeel@rll.berkeley.edu:/var/www/rapprentice"%("--delete" if args.delete else ""), shell=True)
            