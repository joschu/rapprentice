from rapprentice.PR2 import PR2
#roslib.load_manifest("nav_msgs"); import nav_msgs.msg as nm
import trajectory_msgs.msg as tm
import numpy as np
from numpy import sin, cos
import rospy
import scipy.interpolate as si
from Queue import Queue, Empty
from threading import Thread

from time import time, sleep
from numpy import pi

def smaller_ang(x):
    return (x + pi)%(2*pi) - pi

def toRotation(a):
    return np.array([[cos(a), -sin(a)],[sin(a), cos(a)]])

class Spline2D(object):
    def __init__(self,x,y):
        self.Fs = [si.InterpolatedUnivariateSpline(x, ycol) for ycol in y.T]
    def __call__(self, x,nu=0):
        return np.array([F(x,nu=nu) for F in self.Fs]).T

class TrajectoryController:
    def __init__(self):        
        self.brett = PR2()        
        self.sub = rospy.Subscriber("base_traj_controller/command", tm.JointTrajectory, self.callback)
        self.q = Queue()
        self.F = None
        self.stop_requested = False
        self.ctrl_loop_running = False
    def callback(self, msg):
        joints = []
        ts = []
        for jtp in msg.points:
            joints.append((jtp.positions[0], jtp.positions[1], jtp.positions[2]))
            ts.append(jtp.time_from_start.to_sec())
        self.q.put( (np.array(joints),np.array(ts)) )
        self.msg = msg
        
    def listen_loop(self):
        while not rospy.is_shutdown():
            try:
                joints, ts = self.q.get(timeout=.01)
                if self.ctrl_loop_running: self.stop_requested = True
                while self.ctrl_loop_running: sleep(.001)
                ctrl_thread = Thread(target = self.control_loop, args=(joints, ts))
                ctrl_thread.start()
                
            except Empty:
                pass                                                                
    def control_loop(self,joints, ts):
        raise
        
class BaseTrajectoryController(TrajectoryController):
    def control_loop(self,joints,ts):                
        print "running control loop with new trajectory"
        
        
        F = Spline2D(ts, joints)        
        
        t_start = time()        
        duration = ts[-1]

        prev_time = 0

        # xytheta
        # ku=7
        # pu=1.75        
        # kp=0.6*ku
        # ki=2*kp/pu
        # kd=kp*pu/8
        kp = 4.2
        ki = 4.8
        kd = 0.92
        D_state = np.zeros(3)
        I_state = np.zeros(3)
        I_state_max = 1
        I_state_min = -1
        np.set_printoptions(precision=3)
        np.set_printoptions(suppress=True)

        plot = False
        if plot:
            t_plot = []
            x_plot = []
            y_plot = []
            theta_plot = []
            x_targ_plot = []
            y_targ_plot = []
            theta_targ_plot = []

        use_relative = False
        frame_id = self.msg.header.frame_id
        if "base" in frame_id:
            use_relative = True
            pos_start = self.brett.base.get_pose("odom_combined")
        elif "odom_combined" in frame_id or "map" in frame_id:
            pass
        else:
            raise Exception("invalid frame %s for base traj"%frame_id)

        while True:
            
            if rospy.is_shutdown(): 
                return
            if self.stop_requested:
                self.ctrl_loop_running = False
                rospy.loginfo("stop requested--leaving control loop")
                return
            
            t_elapsed = time() - t_start
            if t_elapsed > duration+1:
                rospy.loginfo("time elapsed (+1sec)--leaving control loop")
                if plot:
                    import pylab
                    f1 = pylab.figure()
                    f2 = pylab.figure()
                    f3 = pylab.figure()
                    ax1 = f1.add_axes([0.1,0.1,0.8,0.8])
                    ax1.plot(t_plot, x_plot, t_plot, x_targ_plot)
                    ax2 = f2.add_axes([0.1,0.1,0.8,0.8])
                    ax2.plot(t_plot, y_plot, t_plot, y_targ_plot)
                    ax3 = f3.add_axes([0.1,0.1,0.8,0.8])
                    ax3.plot(t_plot, theta_plot, t_plot, theta_targ_plot)
                    #f1.savefig('/home/jonathan/Desktop/ax1')
                    #f2.savefig('/home/jonathan/Desktop/ax2')
                    #f3.savefig('/home/jonathan/Desktop/ax3')
                return
            
            if t_elapsed > duration: pos_targ = joints[-1].copy()
            else: pos_targ = F(t_elapsed, nu = 0)

            if use_relative:
                # transform target from start frame into odom_combined frame
                pos_targ[:2] = toRotation(pos_start[2]).dot(pos_targ[:2])
                pos_targ += pos_start
            
            # transform target from odom_combined frame to current base frame
            pos_cur = self.brett.base.get_pose("odom_combined")
            pos_targ -= pos_cur
            pos_targ[:2] = toRotation(-pos_cur[2]).dot(pos_targ[:2])
            pos_targ[2] = smaller_ang(pos_targ[2])
            err = pos_targ

            print err

            P_term = kp * err
            D_term = kd * ( err - D_state)/(t_elapsed - prev_time)
            D_state = err
            I_state = I_state + err
            I_state = np.clip(I_state, I_state_min, I_state_max) * (t_elapsed - prev_time)
            I_term = ki * I_state
            twist = P_term + D_term + I_term
            #print P_term, D_term, I_term
            
            # adjust the twist command because of the circular motion
            theta = twist[2]
            if abs(theta) > 0.01:
                factor = theta/(2*np.sin(theta/2))
                twist[:2] *= theta/(2*np.sin(theta/2))
            else:
                factor = 0
            twist[0:2] = np.dot(
                np.array([[np.cos(theta/2), np.sin(theta/2)],
                          [-np.sin(theta/2), np.cos(theta/2)]]) ,
                twist[0:2])

            if plot:
                # use start frame
                if t_elapsed > duration: pos_targ = joints[-1].copy()
                else: pos_targ = F(t_elapsed, nu = 0)
                pos_targ[2] = smaller_ang(pos_targ[2])
                pos_cur -= pos_start
                pos_cur[:2] = toRotation(-pos_start[2]).dot(pos_cur[:2])
                pos_cur[2] = smaller_ang(pos_cur[2])
                t_plot = np.r_[t_plot, t_elapsed]
                x_plot = np.r_[x_plot, pos_cur[0]]
                y_plot = np.r_[y_plot, pos_cur[1]]
                theta_plot = np.r_[theta_plot, pos_cur[2]]
                x_targ_plot = np.r_[x_targ_plot, pos_targ[0]]
                y_targ_plot = np.r_[y_targ_plot, pos_targ[1]]
                theta_targ_plot = np.r_[theta_targ_plot, pos_targ[2]]
            
            self.brett.base.set_twist(twist)
            prev_time = t_elapsed
            sleep(.01)

if __name__ == "__main__":
    import rospy
    rospy.init_node("base_traj_controller", disable_signals = True)
    controller = BaseTrajectoryController()
    controller.listen_loop()