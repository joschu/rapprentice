#!/usr/bin/env ipython -i
from rapprentice.PR2 import PR2
import rospy
rospy.init_node("command_pr2", disable_signals = True)
pr2 = PR2()