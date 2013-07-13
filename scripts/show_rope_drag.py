import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--ropefile")
parser.add_argument("--pert", type=float, default=.1)
parser.add_argument("--seed",type=int,default=0)
args = parser.parse_args()



import cloudprocpy, trajoptpy, openravepy
from rapprentice import berkeley_pr2, PR2, clouds, resampling
import rapprentice.math_utils as mu
import rospy
import h5py
import numpy as np

np.random.seed(args.seed)

from collections import defaultdict
import numpy as np
from rapprentice import math_utils, LOG
import networkx as nx, numpy as np, scipy.spatial.distance as ssd, scipy.interpolate as si
from collections import deque
import itertools
from numpy.random import rand


def voxel_downsample(xyz, s):
    xyz = xyz[np.isfinite(xyz[:,0])]
    d = defaultdict(list)
    for (i,pt) in enumerate(xyz):
        x,y,z = pt
        d[(int(x/s), int(y/s), int(z/s))].append(i)
    return np.array([xyz[inds].mean(axis=0) for inds in d.values()])


def intersect_segs(ps_n2, q_22):
    """Takes a list of 2d nodes (ps_n2) of a piecewise linear curve and two points representing a single segment (q_22)
        and returns indices into ps_n2 of intersections with the segment."""
    assert ps_n2.shape[1] == 2 and q_22.shape == (2, 2)

    def cross(a_n2, b_n2):
        return a_n2[:,0]*b_n2[:,1] - a_n2[:,1]*b_n2[:,0]

    rs = ps_n2[1:,:] - ps_n2[:-1,:]
    s = q_22[1,:] - q_22[0,:]
    denom = cross(rs, s[None,:]) + 1e-10
    qmp = q_22[0,:][None,:] - ps_n2[:-1,:]
    ts = cross(qmp, s[None,:]) / denom # zero denom will make the corresponding element of 'intersections' false
    us = cross(qmp, rs) / denom # same here
    intersections = np.flatnonzero((ts > 0) & (ts < 1) & (us > 0) & (us < 1))
    return intersections, ts, us

def rope_has_intersections(ctl_pts):
    for i in range(len(ctl_pts) - 1):
        curr_seg = ctl_pts[i:i+2,:]
        intersections, ts, us = intersect_segs(ctl_pts[:,:2], curr_seg[:,:2])
        if len(intersections) != 0:
            return True
    return False







########### TOP LEVEL FUNCTION ###############

MIN_SEG_LEN = 3

def find_path_through_point_cloud_simple(xyzs, plotting=False):
    xyzs = np.asarray(xyzs).reshape(-1,3)
    G = points_to_graph(xyzs, .05)
    path = longest_shortest_path(G)
    
    total_path_3d = np.array([G.node[i]["xyz"] for i in path])
    total_path_3d = unif_resample(total_path_3d, seg_len=.02, tol=.003) # tolerance of 1mm
    total_path_3d = unif_resample(total_path_3d, seg_len=.02, tol=.003) # tolerance of 1mm


    return total_path_3d    

def perturb_curve(total_path_3d, perturb_peak_dist, num_perturb_points):
    total_path_3d = np.copy(total_path_3d)
    orig_path_length = np.sqrt(((total_path_3d[1:,:2] - total_path_3d[:-1,:2])**2).sum(axis=1)).sum()
    perturb_centers = np.linspace(0, len(total_path_3d)-1, num_perturb_points).astype(int)
    perturb_xy = np.zeros((len(total_path_3d), 2))
    bandwidth = len(total_path_3d) / (num_perturb_points-1)

    # add a linearly decreasing peak around each perturbation center
    # (keep doing so randomly until our final rope has no loops)
    for _ in range(20):
        for i_center in perturb_centers:
            angle = np.random.rand() * 2 * np.pi
            range_min = max(0, i_center - bandwidth)
            range_max = min(len(total_path_3d), i_center + bandwidth + 1)

            radii = np.linspace(0, perturb_peak_dist, i_center+1-range_min)
            perturb_xy[range_min:i_center+1,:] += np.c_[radii*np.cos(angle), radii*np.sin(angle)]

            radii = np.linspace(perturb_peak_dist, 0, range_max-i_center)
            perturb_xy[i_center+1:range_max,:] += np.c_[radii*np.cos(angle), radii*np.sin(angle)][1:,:]

        unscaled_path_2d = total_path_3d[:,:2] + perturb_xy
        if not rope_has_intersections(unscaled_path_2d):
            break


    total_path_3d[:,:2] = unscaled_path_2d
    total_path_3d = resampling.smooth_positions(total_path_3d, .01)
    return total_path_3d



def longest_shortest_path(G):
    A = nx.floyd_warshall_numpy(G)
    A[np.isinf(A)] = 0
    (i_from_long, i_to_long) = np.unravel_index(A.argmax(), A.shape)
    path = nx.shortest_path(G, source=i_from_long, target=i_to_long)
    return path



def points_to_graph(xyzs, max_dist):
    pdists = ssd.squareform(ssd.pdist(xyzs))
    G = nx.Graph()
    for (i_from, row) in enumerate(pdists):
        G.add_node(i_from, xyz = xyzs[i_from])
        to_inds = np.flatnonzero(row[:i_from] < max_dist)
        for i_to in to_inds:
            G.add_edge(i_from, i_to, length = pdists[i_from, i_to])
    return G


def unif_resample(x,n=None,tol=0,deg=None, seg_len = .02):
 
    if deg is None: deg = min(3, len(x) - 1)

    dl = mu.norms(x[1:] - x[:-1],1)
    l = np.cumsum(np.r_[0,dl])
    goodinds = np.r_[True, dl > 1e-8]

    x = np.atleast_2d(x)
    x = x[goodinds]
    
    (tck,_) = si.splprep(x.T,k=deg,s = tol**2*len(x),u=np.linspace(0,1,len(x)))
    xup = np.array(si.splev(np.linspace(0,1, 10*len(x),.1),tck)).T
    dl = mu.norms(xup[1:] - xup[:-1],1)
    l = np.cumsum(np.r_[0,dl])
    (tck,_) = si.splprep(xup.T,k=deg,s = tol**2*len(xup),u=l)


    if n is not None: newu = np.linspace(0,l[-1],n)
    else: newu = np.linspace(0, l[-1], l[-1]//seg_len)
    return np.array(si.splev(newu,tck)).T    


def largest_connected_component(G):
    sgs = nx.connected_component_subgraphs(G)
    return sgs[0]








def rope_len(x):
    return mu.norms(x[1:] - x[:-1],1).sum()


if args.ropefile:
    perturbed_rope = np.load(args.ropefile)
    demo_rope = None
else:
    hdf = h5py.File("/home/joschu/Data/figure8/figure8.h5")
    demo_xyz = np.asarray(hdf["b0_seg00"]["cloud_xyz"])
    demo_rope = find_path_through_point_cloud_simple(demo_xyz)
    perturbed_rope = perturb_curve(demo_rope, args.pert, 5)
    perturbed_rope = np.array(perturbed_rope)
    center = perturbed_rope.mean(axis=0)
    perturbed_rope = center + (perturbed_rope-center[None,:]) * rope_len(demo_rope) / rope_len(perturbed_rope)
    

np.seterr(all='raise')
rospy.init_node("show_rope_drag")

pr2 = PR2.PR2()
pr2.update_rave()

env = openravepy.Environment()
viewer = trajoptpy.GetViewer(env)





if args.ropefile is None:
    demo_rope_handle = env.plot3(demo_rope, 10, (0,1,1,1))
    perturbed_rope_handle = env.drawlinestrip(perturbed_rope, 10, (1,0,1,1))
else:
    perturbed_rope_handle = env.plot3(perturbed_rope, 16, (0,0,0,1))
    

try:
    grabber = cloudprocpy.CloudGrabber()
    grabber.startRGBD()
    while True:
        rgb, depth = grabber.getRGBD()
        XYZ_k = clouds.depth_to_xyz(depth, berkeley_pr2.f)
        Twk = berkeley_pr2.get_kinect_transform(pr2.robot)
        XYZ_w = XYZ_k.dot(Twk[:3,:3].T) + Twk[:3,3][None,None,:]
        cloud_handle = env.plot3(XYZ_w.reshape(-1,3), 2,  rgb.reshape(-1,3)[:,::-1]/255.)
        viewer.Step()
except:
    import traceback
    traceback.print_exc()
finally:
    grabber.stop()
