#!/usr/bin/env python

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--delete", action="store_true")
args = parser.parse_args()

import subprocess,os
import os.path as osp
import rapprentice
os.chdir(osp.join(osp.dirname(osp.dirname(rapprentice.__file__)),"sampledata"))

print "creating zip file"
import zipfile
with zipfile.ZipFile("all.zip","w") as myzip:
    for fname in os.listdir("."):
        if not (fname.endswith("py") or fname.endswith("zip")):
            myzip.write(fname)
print "uploading"            
subprocess.check_call("rsync -azvu %s ./ pabbeel@rll.berkeley.edu:/var/www/rapprentice/sampledata/ --exclude '*.py'"%("--delete" if args.delete else ""), shell=True)
            