import os
from rapprentice.call_and_print import call_and_print
os.chdir("../scripts")
od="~/Data/rapp2/overhand"
call_and_print("./generate_annotations.py %(od)s/seconddemo_2013-04-17-17-16-39.bag %(od)s/seconddemo_2013-04-17-17-16-39.bag.ann.yaml"%dict(od=od))
call_and_print("./generate_h5.py %(od)s/overhand.yaml"%dict(od=od))
call_and_print("./process_point_clouds.py %(od)s/overhand.h5"%dict(od=od))
