import os
from rapprentice.call_and_print import call_and_print
os.chdir("../scripts")
od="~/henry_sandbox/rapprentice/sampledata/overhand"
call_and_print("./generate_annotations.py %(od)s/demo0_2013-04-20-15-58-09.bag %(od)s/demo0_2013-04-20-15-58-09.bag.ann.yaml"%dict(od=od))
call_and_print("./generate_h5.py %(od)s/overhand.yaml"%dict(od=od))
