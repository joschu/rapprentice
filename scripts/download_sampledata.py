#!/usr/bin/env python
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--rsync",action="store_true")
args = parser.parse_args()

import os, urllib2, zipfile, subprocess
import os.path as osp
import rapprentice
os.chdir(osp.join(osp.dirname(osp.dirname(rapprentice.__file__)),"sampledata"))


if args.rsync: 
    subprocess.check_call("rsync -azvu pabbeel@rll.berkeley.edu:/var/www/rapprentice/sampledata ./ --exclude '*.py'", shell=True)

else:
    print "downloading zip file (this might take a while)"
    urlinfo = urllib2.urlopen("http://rll.berkeley.edu/rapprentice/sampledata/all.zip")
    print "unpacking file"
    with open("all.zip","w") as fh:
        fh.write(urlinfo.read())
    with zipfile.ZipFile("all.zip","r") as myzip:
        myzip.extractall(".")