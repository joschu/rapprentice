import rapprentice, os, os.path as osp
from rapprentice.call_and_print import call_and_print
assert osp.basename(os.getcwd()) == "test"
call_and_print("python tps_unit_tests.py")
call_and_print("python ../scripts/download_sampledata.py ~/Data --use_rsync")
call_and_print("python ../scripts/generate_h5.py ~/Data/sampledata/overhand/overhand.yaml")
call_and_print("python test_registration_synthetic.py --plotting=0")
