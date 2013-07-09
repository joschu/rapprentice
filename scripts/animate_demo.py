#!/usr/bin/env python


"""
Animate demonstration trajectory
"""


import argparse
parser = argparse.ArgumentParser()
parser.add_argument("h5file")
parser.add_argument("--seg")
parser.add_argument("--nopause", action="store_true")
args = parser.parse_args()

import h5py, openravepy,trajoptpy
from rapprentice import animate_traj, ros2rave,clouds
from numpy import asarray
import numpy as np

hdf = h5py.File(args.h5file)

segnames = [args.seg] if args.seg else hdf.keys()

env = openravepy.Environment()
env.StopSimulation()
env.Load("robots/pr2-beta-static.zae")
robot = env.GetRobots()[0]
viewer = trajoptpy.GetViewer(env)


for segname in segnames:

    
    seg_info = hdf[segname]
    
    
    from rapprentice import berkeley_pr2
    
    r2r = ros2rave.RosToRave(robot, seg_info["joint_states"]["name"])
    rave_traj = [r2r.convert(row) for row in asarray(seg_info["joint_states"]["position"])]
    robot.SetActiveDOFs(r2r.rave_inds)
    robot.SetActiveDOFValues(rave_traj[0])
    
    handles = []
    T_w_k = berkeley_pr2.get_kinect_transform(robot)
    o = T_w_k[:3,3]
    x = T_w_k[:3,0]
    y = T_w_k[:3,1]
    z = T_w_k[:3,2]

    handles.append(env.drawarrow(o, o+.3*x, .005,(1,0,0,1)))
    handles.append(env.drawarrow(o, o+.3*y, .005,(0,1,0,1)))
    handles.append(env.drawarrow(o, o+.3*z, .005,(0,0,1,1)))
    

    XYZ_k = clouds.depth_to_xyz(np.asarray(seg_info["depth"]), berkeley_pr2.f)
    Twk = asarray(seg_info["T_w_k"])
    XYZ_w = XYZ_k.dot(Twk[:3,:3].T) + Twk[:3,3][None,None,:]
    RGB = np.asarray(seg_info["rgb"])
    handles.append(env.plot3(XYZ_w.reshape(-1,3), 2,  RGB.reshape(-1,3)[:,::-1]/255.))


    
    animate_traj.animate_traj(rave_traj, robot, pause = not args.nopause)
    
    print "DONE"
    trajoptpy.GetViewer(env).Idle()
    
    
    
    
    
    
