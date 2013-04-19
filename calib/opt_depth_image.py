import argparse
parser = argparse.ArgumentParser()
parser.add_argument("fname")
args = parser.parse_args()


import numpy as np
import itertools
import cv2
from rapprentice import ros2rave
import sensorsimpy, trajoptpy, openravepy
from rapprentice import berkeley_pr2
import scipy.optimize as opt
from rapprentice import clouds

DEBUG_PLOT = True


npzfile = np.load(args.fname)

depth_images = npzfile["depth_images"]
valid_masks = [(d > 0) & ((clouds.depth_to_xyz(d, 525)**2).sum(axis=2) < 1.5**2) for d in depth_images]
depth_images = depth_images/1000.
joint_positions = npzfile["joint_positions"]
joint_names = npzfile["names"] if "names" in npzfile else npzfile["joint_names"]

env = openravepy.Environment()
env.StopSimulation()
env.Load("robots/pr2-beta-static.zae")
robot = env.GetRobots()[0]

viewer = trajoptpy.GetViewer(env)
viewer.Step()
sensor = sensorsimpy.CreateFakeKinect(env)

r2r = ros2rave.RosToRave(robot, joint_names)

rave_inds = r2r.rave_inds
rave_values = np.array([r2r.convert(ros_values) for ros_values in joint_positions])
robot.SetActiveDOFs(rave_inds)
robot.SetActiveDOFValues(rave_values[0])

n_imgs = len(depth_images)
assert len(joint_positions) == n_imgs

MAX_DIST = 1.5

def calc_Thk(xyz, rod):
    T_h_k = openravepy.matrixFromAxisAngle(rod)
    T_h_k[:3,3] += xyz
    return T_h_k

def calculate_sim_depths(xyz, rod, f):
    T_h_k = calc_Thk(xyz, rod)
    T_w_k = robot.GetLink("head_plate_frame").GetTransform().dot(T_h_k)
    
    sensor.SetPose(T_w_k[:3,3], openravepy.quatFromRotationMatrix(T_w_k[:3,:3]))
    sensor.SetIntrinsics(f)    
    out = []
    
    for dofs in rave_values:
        robot.SetActiveDOFValues(dofs)
        viewer.UpdateSceneData()
        sensor.Update()
        out.append(sensor.GetDepthImage())
        
    return out
    
def calculate_depth_error(xyz, rod, f):
    errs = np.empty(n_imgs)
    sim_depths = calculate_sim_depths(xyz, rod, f)
    for i in xrange(n_imgs):
        sim_depth = sim_depths[i]
        real_depth = depth_images[i]
        valid_mask = valid_masks[i]
        errs[i] = (real_depth - sim_depth)[valid_mask].__abs__().sum()
                
    return errs.sum()

def plot_real_and_sim(x):
    print x
    sim_depths = calculate_sim_depths(*vec2args(x))
    for i in xrange(1):
        sim_depth = sim_depths[i]
        real_depth = depth_images[i]
        cv2.imshow("sim%i"%i, sim_depth/MAX_DIST)
        cv2.imshow("real%i"%i, real_depth/MAX_DIST)
        cv2.imshow("valid%i"%i, valid_masks[i].astype('uint8')*255)        
        cv2.moveWindow("sim%i"%i, 0, 0)
        cv2.moveWindow("real%i"%i, 640, 0)
        cv2.moveWindow("valid%i"%i, 1280, 0)
    #print "press key to continue"
    cv2.waitKey(10)
        
    
def calc_error_wrapper(x):
    return calculate_depth_error(*vec2args(x))
def vec2args(x):
    return x[0:3], x[3:6], x[6]
def args2vec(xyz, rod, f):
    out = np.empty(7)
    out[0:3] = xyz
    out[3:6] = rod
    out[6] = f
    return out

T_w_k = berkeley_pr2.get_kinect_transform(robot)

#o = T_w_k[:3,3]
#x = T_w_k[:3,0]
#y = T_w_k[:3,1]
#z = T_w_k[:3,2]
#handles = []
#handles.append(env.drawarrow(o, o+.2*x, .005,(1,0,0,1)))
#handles.append(env.drawarrow(o, o+.2*y, .005,(0,1,0,1)))
#handles.append(env.drawarrow(o, o+.2*z, .005,(0,0,1,1)))
#viewer.Idle()


T_h_k_init = berkeley_pr2.T_h_k

xyz_init = T_h_k_init[:3,3]
rod_init = openravepy.axisAngleFromRotationMatrix(T_h_k_init[:3,:3])
f_init = 525

#from rapprentice.math_utils import linspace2d



#sim_depths = calculate_sim_depths(xyz_init, rod_init, f_init)    
#cv2.imshow('hi',np.clip(sim_depths[0]/3,0,1))
#cv2.waitKey()
#plot_real_and_sim(args2vec(xyz_init, rod_init, f_init))
#calculate_depth_error(xyz_init, rod_init, f_init)
soln = opt.fmin(calc_error_wrapper, args2vec(xyz_init, rod_init, f_init),callback=plot_real_and_sim)
(best_xyz, best_rod, best_f) = vec2args(soln)
print "xyz, rod:", best_xyz, best_rod
print "T_h_k:", calc_Thk(best_xyz, best_rod)
print "f:",best_f