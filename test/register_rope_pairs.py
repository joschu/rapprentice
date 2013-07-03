import h5py
import rapprentice.registration as reg, rapprentice.cloud_proc_funcs as cpf
from joblib import Memory, Parallel, delayed
import cPickle, h5py, os.path as osp, numpy as np, scipy.spatial.distance as ssd
from time import time

aa = np.asarray
mem = Memory(cachedir='/tmp/joblib')

cached_extract_red = mem.cache(cpf.extract_red)

@mem.cache(ignore=["hdf"])
def extract_clouds(hdf):
    return [cached_extract_red(aa(seg["rgb"]), aa(seg["depth"]), aa(seg["T_w_k"])) for (key,seg) in hdf.items()]

if __name__ == "__main__":
    hdf = h5py.File("/Users/joschu/Data/knots/master.h5", "r")
    clouds = extract_clouds(hdf)    


    dim = 2
    

    for cloud0 in clouds:
        for cloud1 in clouds:
            if dim ==2 :
                cloud0 = cloud0[:,:2]
                cloud1 = cloud1[:,:2]
            scaled_cloud0, src_params = reg.unit_boxify(cloud0)
            scaled_cloud1, targ_params = reg.unit_boxify(cloud1)
    
            print "downsampled to %i and %i pts"%(len(scaled_cloud0),len(scaled_cloud1))
    
            tstart = time()
            fest_scaled,gest_scaled = reg.tps_rpm_bij(scaled_cloud0, scaled_cloud1, n_iter=10, reg_init = 10, reg_final=.01)
            print "%.3f elapsed"%(time() - tstart)
            
            cost = reg.tps_reg_cost(fest_scaled) + reg.tps_reg_cost(gest_scaled)
            print "cost: %.3f"%cost

            import matplotlib.pyplot as plt
            plt.clf()
            # plt.plot(xys[:,1], xys[:,0],'r.')
            # plt.plot(fxys[:,1], fxys[:,0],'b.')
            # plt.plot(fxys_est[:,1], fxys_est[:,0],'g.')

            scaled_fcloud = fest_scaled.transform_points(scaled_cloud0)
            plt.plot(scaled_cloud0[:,1], scaled_cloud0[:,0],'r.')
            plt.plot(scaled_cloud1[:,1], scaled_cloud1[:,0],'b.')
            plt.plot(scaled_fcloud[:,1], scaled_fcloud[:,0],'g.')
            
            
            if dim == 2: 
                from rapprentice.plotting_plt import plot_warped_grid_2d
                plot_warped_grid_2d(fest_scaled.transform_points, [-.5,-.5], [.5,.5])
            
            
            elif dim == 3:
                env= openravepy.RaveGetEnvironment(1)
                if env is None: env = openravepy.Environment()
                from rapprentice.plotting_openrave import draw_grid
                draw_grid(fest_scaled, [-.5,-.5,-.5], [.5,.5,.5])
            
            plt.draw()
            raw_input()
