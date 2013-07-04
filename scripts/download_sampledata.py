#!/usr/bin/env python
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("path", help="script will create a sampledata directory in this location")
parser.add_argument("--use_rsync", action="store_true")
args = parser.parse_args()

import os, urllib2, shutil
from rapprentice.yes_or_no import yes_or_no
from rapprentice.call_and_print import call_and_print

if args.use_rsync:
    call_and_print("rsync -azvu --delete pabbeel@rll.berkeley.edu:/var/www/rapprentice/sampledata %s"%args.path)

else: 
    os.chdir(args.path)
    if os.path.exists("sampledata"):
        yn = yes_or_no("delete old directory")
        if yn: shutil.rmtree("sampledata")
        else: raise IOError
    os.mkdir("sampledata")

    print "downloading archive file"
    urlinfo = urllib2.urlopen("http://rll.berkeley.edu/rapprentice/sampledata.tar")
    with open("sampledata.tar","w") as fh:
        fh.write(urlinfo.read())
    print "unpacking file"
    call_and_print("tar xvf sampledata.tar")
    os.unlink("sampledata.tar")