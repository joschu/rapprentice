#!/bin/sh
pylint -f colorized --disable=C,R -r n rapprentice
cd scripts
pylint -f colorized --disable=C,R -r n *.py
cd ..
