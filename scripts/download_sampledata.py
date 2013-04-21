#!/usr/bin/env python
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("path", help="script will create a sampledata directory in this location")
args = parser.parse_args()

import os, urllib2, subprocess, shutil
import os.path as osp
import rapprentice
from rapprentice.yes_or_no import yes_or_no
os.chdir(args.path)
if os.path.exists("sampledata"):
    yn = yes_or_no("delete old directory")
    if yn: shutil.rmtree("sampledata")
    else: raise IOError
os.mkdir("sampledata")
os.chdir("sampledata")

print "downloading zip file"
urlinfo = urllib2.urlopen("http://rll.berkeley.edu/rapprentice/sampledata/all.tar")
with open("all.tar","w") as fh:
    fh.write(urlinfo.read())
print "unpacking file"
subprocess.check_call("tar xvf all.tar", shell=True)
os.unlink("all.tar")