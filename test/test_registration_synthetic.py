import argparse, os.path as osp
parser = argparse.ArgumentParser()
parser.add_argument("--mode",choices=["self","pairwise"],default="self")
parser.add_argument("--sampledata_dir",default=osp.expanduser("~/Data/sampledata"))
parser.add_argument("--plotting", type=int,default=1)
parser.add_argument("--to3d", action="store_true")
args = parser.parse_args()

from collections import defaultdict
import cv2, numpy as np
from rapprentice.plotting_plt import plot_warped_grid_2d
import rapprentice.registration as reg
from time import time
import os, os.path as osp
from glob import glob

def pixel_downsample(xy, s):
    xy = xy[np.isfinite(xy[:,0])]
    d = defaultdict(list)
    for (i,pt) in enumerate(xy):
        x,y = pt
        d[(int(x/s), int(y/s))].append(i)
    return np.array([xy[inds].mean(axis=0) for inds in d.values()])
def voxel_downsample(xyz, s):
    xyz = xyz[np.isfinite(xyz[:,0])]
    d = defaultdict(list)
    for (i,pt) in enumerate(xyz):
        x,y,z = pt
        d[(int(x/s), int(y/s), int(z/s))].append(i)
    return np.array([xyz[inds].mean(axis=0) for inds in d.values()])


def registration(xys, fxys):
    if args.to3d:
        xys = np.c_[xys, np.zeros((len(xys)))]
        fxys = np.c_[fxys, np.zeros((len(fxys)))]

    
    scaled_xys, src_params = reg.unit_boxify(xys)
    scaled_fxys, targ_params = reg.unit_boxify(fxys)
    
    downsample = voxel_downsample if args.to3d else pixel_downsample
    scaled_ds_xys = downsample(scaled_xys, .03)
    scaled_ds_fxys = downsample(scaled_fxys, .03)
    
    print "downsampled to %i and %i pts"%(len(scaled_ds_xys),len(scaled_ds_fxys))
    tstart = time()
    
    # fest_scaled = reg.tps_rpm(scaled_ds_xys, scaled_ds_fxys, n_iter=10, reg_init = 10, reg_final=.01)
    fest_scaled,_ = reg.tps_rpm_bij(scaled_ds_xys, scaled_ds_fxys, n_iter=10, reg_init = 10, reg_final=.01)
    
    
    
    print "time: %.4f"%(time()-tstart)
    fest = reg.unscale_tps(fest_scaled, src_params, targ_params)
    fxys_est = fest.transform_points(xys)
    if len(fxys_est) == len(fxys): print "error:", np.abs(fxys_est - fxys).mean()
    
    if args.plotting:
        import matplotlib.pyplot as plt
        plt.clf()
        # plt.plot(xys[:,1], xys[:,0],'r.')
        # plt.plot(fxys[:,1], fxys[:,0],'b.')
        # plt.plot(fxys_est[:,1], fxys_est[:,0],'g.')

        scaled_ds_fxys_est = fest_scaled.transform_points(scaled_ds_xys)
        plt.plot(scaled_ds_xys[:,1], scaled_ds_xys[:,0],'r.')
        plt.plot(scaled_ds_fxys[:,1], scaled_ds_fxys[:,0],'b.')
        plt.plot(scaled_ds_fxys_est[:,1], scaled_ds_fxys_est[:,0],'g.')


        def to2d(f):
            def f2d(x):
                return f(np.c_[x, np.zeros((len(x),1))])[:,:2]
            return f2d
        transform_func = to2d(fest_scaled.transform_points) if args.to3d else fest_scaled.transform_points
        plot_warped_grid_2d(transform_func, [-.5,-.5], [.5,.5])
        plt.draw()
        plt.ginput()
        
def main():
    
    assert osp.exists(osp.join(args.sampledata_dir, "letter_images"))

    fnames = glob(osp.join(args.sampledata_dir, "letter_images", "*.png"))
    
    if args.mode == "self":
        for fname in fnames:    
            im = cv2.imread(fname, 0)
            assert im is not None
            xs, ys = np.nonzero(im==0)
    
            
            xys = np.c_[xs, ys]
            R = np.array([[.7, .4], [.3,.9]])
            T = np.array([[.3,.4]])
            fxys = (xys.dot(R) + T)**2 
            
            xys[:,0] *= -1
            fxys[:,0] *= -1
            
            registration(xys, fxys)
    
    else:
        import itertools
        for (fname0, fname1) in itertools.combinations(fnames, 2):
            if osp.basename(fname0)[0] == osp.basename(fname1)[0]:
                print "registering %s to %s"%(fname0, fname1)

                im0 = cv2.imread(fname0, 0)
                im1 = cv2.imread(fname1, 0)
                assert im0 is not None and im1 is not None
                xs, ys = np.nonzero(im0==0)
                fxs, fys = np.nonzero(im1==0)
                xs = xs * -1
                fxs = fxs * -1

    
                xys = np.c_[xs, ys]
                fxys = np.c_[fxs, fys]
                
                registration(xys, fxys)
if __name__ == "__main__":
    main()
