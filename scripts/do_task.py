#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("h5file", type=str)
parser.add_argument("--cloud_proc_func", default="extract_red")
parser.add_argument("--cloud_proc_mod", default="rapprentice.cloud_proc_funcs")
    
parser.add_argument("--execution", type=int, default=0)
parser.add_argument("--animation", type=int, default=0)

parser.add_argument("--fake_data_segment",type=str)
parser.add_argument("--fake_data_transform", type=float, nargs=6, metavar=("tx","ty","tz","rx","ry","rz"),
    default=[0,0,0,0,0,0], help="translation=(tx,ty,tz), axis-angle rotation=(rx,ry,rz)")

parser.add_argument("--interactive",action="store_true")

args = parser.parse_args()

if args.fake_data_segment is None: assert args.execution==1

###################


"""
Workflow:
1. Fake data + animation only
    --fake_data_segment=xxx --execution=0
2. Fake data + Gazebo. Set Gazebo to initial state of fake data segment so we'll execute the same thing.
    --fake_data_segment=xxx --execution=1
    This is just so we know the robot won't do something stupid that we didn't catch with openrave only mode.
3. Real data + Gazebo
    --execution=1 
    The problem is that the gazebo robot is in a different state from the real robot, in particular, the head tilt angle. TODO: write a script that       sets gazebo head to real robot head
4. Real data + Real execution.
    --execution=1

The question is, do you update the robot's head transform.
If you're using fake data, don't update it.

"""


from rapprentice import registration, colorize, berkeley_pr2, \
     animate_traj, ros2rave, plotting_openrave, task_execution
from rapprentice import pr2_trajectories, PR2
import rospy

import cloudprocpy, trajoptpy, json, openravepy
import os, numpy as np, h5py, os.path as osp
from numpy import asarray
import importlib
import subprocess

cloud_proc_mod = importlib.import_module(args.cloud_proc_mod)
cloud_proc_func = getattr(cloud_proc_mod, args.cloud_proc_func)
    

    
    
def redprint(msg):
    print colorize.colorize(msg, "red", bold=True)
    
def split_trajectory_by_gripper(seg_info):
    rgrip = asarray(seg_info["r_gripper_joint"])
    lgrip = asarray(seg_info["l_gripper_joint"])

    thresh = .04 # open/close threshold

    n_steps = len(lgrip)


    # indices BEFORE transition occurs
    l_openings = np.flatnonzero((lgrip[1:] >= thresh) & (lgrip[:-1] < thresh))
    r_openings = np.flatnonzero((rgrip[1:] >= thresh) & (rgrip[:-1] < thresh))
    l_closings = np.flatnonzero((lgrip[1:] < thresh) & (lgrip[:-1] >= thresh))
    r_closings = np.flatnonzero((rgrip[1:] < thresh) & (rgrip[:-1] >= thresh))

    before_transitions = np.r_[l_openings, r_openings, l_closings, r_closings]
    after_transitions = before_transitions+1
    seg_starts = np.unique(np.r_[0, after_transitions])
    seg_ends = np.unique(np.r_[before_transitions, n_steps-1])

    return seg_starts, seg_ends

def binarize_gripper(angle):
    open_angle = .08
    closed_angle = 0    
    thresh = .04
    if angle > thresh: return open_angle
    else: return closed_angle


    
def plan_follow_traj(robot, manip_name, ee_link, new_hmats, old_traj):
        
    n_steps = len(new_hmats)
    assert old_traj.shape[0] == n_steps
    assert old_traj.shape[1] == 7
    
    arm_inds  = robot.GetManipulator(manip_name).GetArmIndices()

    ee_linkname = ee_link.GetName()
    
    init_traj = old_traj.copy()
    #init_traj[0] = robot.GetDOFValues(arm_inds)

    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : manip_name,
            "start_fixed" : False
        },
        "costs" : [
        {
            "type" : "joint_vel",
            "params": {"coeffs" : [1]}
        },
        {
            "type" : "collision",
            "params" : {"coeffs" : [10],"dist_pen" : [0.005]}
        }                
        ],
        "constraints" : [
        ],
        "init_info" : {
            "type":"given_traj",
            "data":[x.tolist() for x in init_traj]
        }
    }
        
    poses = [openravepy.poseFromMatrix(hmat) for hmat in new_hmats]
    for (i_step,pose) in enumerate(poses):
        request["costs"].append(
            {"type":"pose",
             "params":{
                "xyz":pose[4:7].tolist(),
                "wxyz":pose[0:4].tolist(),
                "link":ee_linkname,
                "timestep":i_step,
                "pos_coeffs":[20,20,20],
                "rot_coeff":[20,20,20]
             }
            })

    s = json.dumps(request)
    prob = trajoptpy.ConstructProblem(s, Globals.env) # create object that stores optimization problem
    result = trajoptpy.OptimizeProblem(prob) # do optimization
    traj = result.GetTraj()    
        
    saver = openravepy.RobotStateSaver(robot)
    pos_errs = []
    for i_step in xrange(1,n_steps):
        row = traj[i_step]
        robot.SetDOFValues(row, arm_inds)
        tf = ee_link.GetTransform()
        pos = tf[:3,3]
        pos_err = np.linalg.norm(poses[i_step][4:7] - pos)
        pos_errs.append(pos_err)
    pos_errs = np.array(pos_errs)
        
    print "planned trajectory for %s. max position error: %.3f. all position errors: %s"%(manip_name, pos_errs.max(), pos_errs)
            
    return traj         
    
def set_gripper_maybesim(lr, value):
    if args.execution:
        gripper = {"l":Globals.pr2.lgrip, "r":Globals.pr2.rgrip}[lr]
        gripper.set_angle(value)
        Globals.pr2.join_all()
    else:
        Globals.robot.SetDOFValues([value*5], [Globals.robot.GetJoint("%s_gripper_l_finger_joint"%lr).GetDOFIndex()])
        
def exec_traj_maybesim(bodypart2traj):
    if args.animation:
        dof_inds = []
        trajs = []
        for (part_name, traj) in bodypart2traj.items():
            manip_name = {"larm":"leftarm","rarm":"rightarm"}[part_name]
            dof_inds.extend(Globals.robot.GetManipulator(manip_name).GetArmIndices())            
            trajs.append(traj)
        full_traj = np.concatenate(trajs, axis=1)
        Globals.robot.SetActiveDOFs(dof_inds)
        animate_traj.animate_traj(full_traj, Globals.robot, restore=False,pause=True)
    if args.execution:
        pr2_trajectories.follow_body_traj(Globals.pr2, bodypart2traj)



def find_closest(demofile, new_xyz):
    "for now, just prompt the user"
    seg_names = demofile.keys()
    print "choose from the following options (type an integer)"
    for (i, seg_name) in enumerate(seg_names):
        print "%i: %s"%(i,seg_name)
    choice_ind = task_execution.request_int_in_range(len(seg_names))
    chosen_seg = seg_names[choice_ind] 
    return chosen_seg
            
            
def arm_moved(joint_traj):    
    return ((joint_traj[1:] - joint_traj[:-1]).ptp(axis=0) > .05).any()
        
def tpsrpm_plot_cb(x_nd, y_md, targ_Nd, corr_nm, wt_n, f):
    ypred_nd = f.transform_points(x_nd)
    handles = []
    handles.append(Globals.env.plot3(ypred_nd, 3, (0,1,0)))
    handles.extend(plotting_openrave.draw_grid(Globals.env, f.transform_points, x_nd.min(axis=0), x_nd.max(axis=0), xres = .1, yres = .1, zres = .04))
    Globals.viewer.Step()
###################


class Globals:
    robot = None
    env = None

    pr2 = None

def main():

    demofile = h5py.File(args.h5file, 'r')
    
    trajoptpy.SetInteractive(args.interactive)

    if args.execution:
        rospy.init_node("exec_task",disable_signals=True)
        Globals.pr2 = PR2.PR2()
        Globals.env = Globals.pr2.env
        Globals.robot = Globals.pr2.robot
        
    else:
        Globals.env = openravepy.Environment()
        Globals.env.StopSimulation()
        Globals.env.Load("robots/pr2-beta-static.zae")    
        Globals.robot = Globals.env.GetRobots()[0]

    if not args.fake_data_segment:
        grabber = cloudprocpy.CloudGrabber()
        grabber.startRGBD()

    Globals.viewer = trajoptpy.GetViewer(Globals.env)

    #####################

    while True:
        
    
        redprint("Acquire point cloud")
        if args.fake_data_segment:
            new_xyz = np.squeeze(demofile[args.fake_data_segment]["cloud_xyz"])
            hmat = openravepy.matrixFromAxisAngle(args.fake_data_transform[3:6])
            hmat[:3,3] = args.fake_data_transform[0:3]
            new_xyz = new_xyz.dot(hmat[:3,:3].T) + hmat[:3,3][None,:]
            
        else:
            
            Globals.pr2.rarm.goto_posture('side')
            Globals.pr2.larm.goto_posture('side')            
            Globals.pr2.join_all()
            
            Globals.pr2.update_rave()            
            
            rgb, depth = grabber.getRGBD()
            T_w_k = berkeley_pr2.get_kinect_transform(Globals.robot)
            new_xyz = cloud_proc_func(rgb, depth, T_w_k)

    
        ################################    
        redprint("Finding closest demonstration")
        if args.fake_data_segment:
            seg_name = args.fake_data_segment
        else:
            seg_name = find_closest(demofile, new_xyz)
        
        seg_info = demofile[seg_name]
        # redprint("using demo %s, description: %s"%(seg_name, seg_info["description"]))
    
        ################################

        redprint("Generating end-effector trajectory")    

        handles = []
        old_xyz = np.squeeze(seg_info["cloud_xyz"])
        handles.append(Globals.env.plot3(old_xyz,5, (1,0,0,1)))
        handles.append(Globals.env.plot3(new_xyz,5, (0,0,1,1)))


        f = registration.tps_rpm(old_xyz, new_xyz, plot_cb = tpsrpm_plot_cb,plotting=1)            
        
        #f = registration.ThinPlateSpline() XXX XXX
        
        handles.extend(plotting_openrave.draw_grid(Globals.env, f.transform_points, old_xyz.min(axis=0), old_xyz.max(axis=0), xres = .1, yres = .1, zres = .04))
        

        link2eetraj = {}
        for lr in 'lr':
            link_name = "%s_gripper_tool_frame"%lr
            old_ee_traj = asarray(seg_info[link_name]["hmat"])
            new_ee_traj = f.transform_hmats(old_ee_traj)
            link2eetraj[link_name] = new_ee_traj
            
            handles.append(Globals.env.drawlinestrip(old_ee_traj[:,:3,3], 2, (1,0,0,1)))
            handles.append(Globals.env.drawlinestrip(new_ee_traj[:,:3,3], 2, (0,1,0,1)))
            
            
            
        # TODO plot
        # plot_warping_and_trajectories(f, old_xyz, new_xyz, old_ee_traj, new_ee_traj)
    
        miniseg_starts, miniseg_ends = split_trajectory_by_gripper(seg_info)    
        success = True
        print colorize.colorize("mini segments:", "red"), miniseg_starts, miniseg_ends
        for (i_miniseg, (i_start, i_end)) in enumerate(zip(miniseg_starts, miniseg_ends)):
            
            if args.execution=="real": Globals.pr2.update_rave()


            ################################    
            redprint("Generating joint trajectory for segment %s, part %i"%(seg_name, i_miniseg))

            bodypart2traj = {}
            
            arms_used = ""
        
            for lr in 'lr':
                manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
                old_joint_traj = asarray(seg_info[manip_name][i_start:i_end+1])
                if arm_moved(old_joint_traj):          
                    ee_link_name = "%s_gripper_tool_frame"%lr
                    new_ee_traj = link2eetraj[ee_link_name]
                    if args.execution: Globals.pr2.update_rave()                    
                    new_joint_traj = plan_follow_traj(Globals.robot, manip_name,
                     Globals.robot.GetLink(ee_link_name), new_ee_traj[i_start:i_end+1], 
                     old_joint_traj)
                    # (robot, manip_name, ee_link, new_hmats, old_traj):
                    part_name = {"l":"larm", "r":"rarm"}[lr]
                    bodypart2traj[part_name] = new_joint_traj
                    arms_used += lr

        

            ################################    
            redprint("Executing joint trajectory for segment %s, part %i using arms '%s'"%(seg_name, i_miniseg, arms_used))

            for lr in 'lr':
                set_gripper_maybesim(lr, binarize_gripper(seg_info["%s_gripper_joint"%lr][i_start]))
            #trajoptpy.GetViewer(Globals.env).Idle()
        
            if len(bodypart2traj) > 0:
                exec_traj_maybesim(bodypart2traj)
        
            # TODO measure failure condtions

            if not success:
                break
            
        redprint("Segment %s result: %s"%(seg_name, success))
    
    
        if args.fake_data_segment: break


main()
