import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--group")
args = parser.parse_args()

import yaml
import numpy as np
from lfd import registration, tps
import os.path as osp

tps.VERBOSE=False
np.seterr(all='raise')    
np.set_printoptions(precision=3)

def axisanglepart(m):
    if np.linalg.det(m) < 0 or np.isnan(m).any(): raise Exception
    u,s,vh = np.linalg.svd(m)
    rotpart = u.dot(vh)
    theta = np.arccos((np.trace(rotpart) - 1)/2)   
    print "theta", theta
    return (1/(2*np.sin(theta))) * np.array([[rotpart[2,1] - rotpart[1,2], rotpart[0,2]-rotpart[2,0], rotpart[1,0]-rotpart[0,1]]])


with open("ground_truth.yaml","r") as fh:
    gtdata = yaml.load(fh)
    
grouplist =   gtdata if args.group is None else [group for group in gtdata if group["name"] == args.group]  
    
for group in grouplist:
    print "group: ", group["name"]
    xyz_base = np.loadtxt(osp.join("../test/test_pcs",group["base"]))
    
    def plot_cb(x_nd, y_md, targ_nd, corr_nm, wt_n, f):
        return
        #return
        from mayavi import mlab
        fignum=0
        fignum = mlab.figure(0)
        mlab.clf(figure=fignum)
        x,y,z = x_nd.T
        mlab.points3d(x,y,z, color=(1,0,0),scale_factor=.01,figure=fignum)
        x,y,z, = f.transform_points(x_nd).T
        mlab.points3d(x,y,z, color=(0,1,0),scale_factor=.01,figure=fignum)
        x,y,z =  y_md.T
        mlab.points3d(x,y,z, color=(0,0,1),scale_factor=.01,figure=fignum)        
        #raw_input()
    
    for other in group["others"]:
        xyz_other = np.loadtxt(osp.join("../test/test_pcs", other["file"]))
        f = registration.tps_rpm_zrot(xyz_base, xyz_other,reg_init=2,reg_final=.5,n_iter=9, 
                                      verbose=True, rot_param = (0.01,0.01,0.0025),scale_param=.01, plot_cb = plot_cb, plotting=1)
        T_calc = np.c_[ f.lin_ag.T, f.trans_g.reshape(3,1) ] # (.01,.01,.005)
        print "result"
        print T_calc
        print "axis-angle:",
        print axisanglepart(T_calc[:3,:3])
        if "transform" in other:
            T_other_base = np.array(other["transform"]).reshape(4,4)
            print "actual"
            print T_other_base

        import transform_gui
        transformer = transform_gui.CloudAffineTransformer(xyz_base, xyz_other, T_calc)
        transformer.configure_traits()
