#!/bin/sh

if [ `uname` = "Darwin" ]
then
    disabled="C,R,F0401,W0142,W1401"
else
    disabled="C,R,F0401,W0142"
fi
pylint -f colorized --disable=$disabled -r n -i y rapprentice | grep -v openravepy
cd scripts
pylint -f colorized --disable=$disabled -r n -i y  *.py | grep -v openravepy
cd ..
