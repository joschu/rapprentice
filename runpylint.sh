#!/bin/sh

disabled="C,R,F0401,W0142,W1401"
pylint -f colorized --disable=$disabled -r n -i y rapprentice | grep -v openravepy
cd scripts
pylint -f colorized --disable=$disabled -r n -i y  *.py | grep -v openravepy
cd ..
