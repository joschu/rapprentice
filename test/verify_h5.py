import h5py

parser = argparse.ArgumentParser()
parser.add_argument("h5file")
args = parser.parse_args()


import argparse

hdf = h5py.File(args.h5file)

env = openravepy.Environment()
env.Load("robots/pr2-beta-static.zae")
robot = env.GetRobots()[0]

for (seg_name, seg_info)  in hdf.items():
    pass