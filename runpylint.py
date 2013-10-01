#!/usr/bin/env python
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--files",nargs="+")
args = parser.parse_args()

import subprocess,os,sys


if os.uname()[0] == "Darwin":
    disabled="C,R,F0401,W0142,W1401"
else:
    disabled="C,R,F0401,W0142"


if args.files is not None:
    for file in args.files:
        subprocess.call("pylint -f colorized --disable=%s -r n -i y %s | grep -v openravepy"%(disabled,file),shell=True)
else:
    subprocess.call("pylint -f colorized --disable=%s -r n -i y rapprentice | grep -v openravepy"%disabled, shell=True)
    os.chdir("scripts")
    subprocess.call("pylint -f colorized --disable=%s -r n -i y  *.py | grep -v openravepy"%disabled, shell=True)
    os.chdir("..")

