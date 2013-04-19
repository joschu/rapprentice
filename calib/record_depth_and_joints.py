import cloudprocpy
import rospy
from rapprentice.PR2 import TopicListener
import sensor_msgs.msg as sm
import numpy as np


rospy.init_node("record_d_and_j", disable_signals = True)


grabber = cloudprocpy.CloudGrabber()
grabber.startRGBD()

joint_listener = TopicListener("/joint_states", sm.JointState)

depth_images = []
joint_positions = []


while True:
    exit_requested = False
    while True:
        
        print "press 'a' to acquire image. press 'e' to exit"
        k = raw_input()
        if k == 'a':
            break
        elif k == 'e':
            exit_requested=True
            break
        else:
            print "invalid selection"
    if exit_requested:
        break
    
    joint_positions.append(joint_listener.last_msg.position)
    _, d = grabber.getRGBD()
    depth_images.append(d)


fname = "/tmp/calib_data.npz"    
print "saving to %s"%fname
np.savez(fname, depth_images=depth_images, joint_positions=joint_positions, joint_names = joint_listener.last_msg.name)
