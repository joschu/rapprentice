"""
Resample time serieses to reduce the number of datapoints
"""
from __future__ import division
import numpy as np
from rapprentice import LOG
import rapprentice.math_utils as mu
import fastrapp
import scipy.interpolate as si
import openravepy

def lerp(a, b, fracs):
    "linearly interpolate between a and b"
    fracs = fracs[:,None]
    return a*(1-fracs) + b*fracs

def adaptive_resample(x, t = None, max_err = np.inf, max_dx = np.inf, max_dt = np.inf, normfunc = None):
    """
    SLOW VERSION
    """
    #return np.arange(0, len(x), 100)
    LOG.info("resampling a path of length %i", len(x))
    x = np.asarray(x)
    
    if x.ndim == 1: x = x[:,None]
    else: assert x.ndim == 2
    
    if normfunc is None: normfunc = np.linalg.norm
    
    if t is None: t = np.arange(len(x))
    else: t = np.asarray(t)
    assert(t.ndim == 1)
    
    n = len(t)
    assert len(x) == n

    import networkx as nx
    g = nx.DiGraph()
    g.add_nodes_from(xrange(n))
    for i_src in xrange(n):
        for i_targ in xrange(i_src+1, n):
            normdx = normfunc(x[i_targ] - x[i_src])
            dt = t[i_targ] - t[i_src]
            errs = lerp(x[i_src], x[i_targ], (t[i_src:i_targ+1] - t[i_src])/dt) - x[i_src:i_targ+1]
            interp_err = errs.__abs__().max()#np.array([normfunc(err) for err in errs]).max()
            if normdx > max_dx or dt > max_dt or interp_err > max_err: break
            g.add_edge(i_src, i_targ)
    resample_inds = nx.shortest_path(g, 0, n-1)
    return resample_inds


def get_velocities(positions, times, tol):
    positions = np.atleast_2d(positions)
    n = len(positions)
    deg = min(3, n - 1)
    
    good_inds = np.r_[True,(abs(times[1:] - times[:-1]) >= 1e-6)]
    good_positions = positions[good_inds]
    good_times = times[good_inds]
    
    if len(good_inds) == 1:
        return np.zeros(positions[0:1].shape)
    
    (tck, _) = si.splprep(good_positions.T,s = tol**2*(n+1), u=good_times, k=deg)
    #smooth_positions = np.r_[si.splev(times,tck,der=0)].T
    velocities = np.r_[si.splev(times,tck,der=1)].T    
    return velocities

def smooth_positions(positions, tol):
    times = np.arange(len(positions))
    positions = np.atleast_2d(positions)
    n = len(positions)
    deg = min(3, n - 1)
    
    good_inds = np.r_[True,(abs(times[1:] - times[:-1]) >= 1e-6)]
    good_positions = positions[good_inds]
    good_times = times[good_inds]
    
    if len(good_inds) == 1:
        return np.zeros(positions[0:1].shape)
    
    (tck, _) = si.splprep(good_positions.T,s = tol**2*(n+1), u=good_times, k=deg)
    spos = np.r_[si.splev(times,tck,der=0)].T
    return spos


def unif_resample3(Y, n, x = None, weights=None, tol=.0001, deg=3, n_length_eval=300):
    """
    interpolate the function Y(x)
    
    
    Input:
    Y:   ndarray with shape (T,K) 
    n:   integer number of samples
    x:   ndarray with shape (T,). Optional: if omitted, it's assuped to be arange(len(Y))

    tol: informally, it's the acceptable error in the interpolated function.         
         interpolating spline Y computed internally will satisfy
          \sum_i | Y(x_i) - Y_i |^2  <= T * tol^2

    weights: not implemented. but you may want to take a measure of path-length
         that weights different components of Y differently.
    deg:  degree of spline
    n_length_eval: higher -> result is more precisely uniform
    
    
    Returns:
    Yunif   resampled Y values
    xunif   resampled x values   
    
    So if you plot (xunif, Yunif[:,i])
    it should look like the graph of (x,Y[:,i])
    
    Yunif approximately has norm(Yunif[i+1] - Yunif[i]) constant for i=0,1,2,...
    
    """
    if Y.ndim == 1: Y = Y[:,None]
    if x is None: x = np.arange(len(Y))
    if weights is not None: raise NotImplementedError
    
    goodinds = np.r_[0,1+np.flatnonzero(x[1:]-x[:-1] > 1e-10)] 
    splines = [si.UnivariateSpline(x[goodinds], y, s = tol**2 * (n+1)) for y in Y[goodinds].T]
    
    xup = np.linspace(0,x[-1],n_length_eval) # For evaluating path length integral
    
    def _eval_splines(_splines, _x):
        return np.array([spline(_x) for spline in splines]).T
        
    Yup = _eval_splines(splines, xup)
    

    dl = mu.norms( Yup[1:] - Yup[:-1], 1)
    l = np.empty(n_length_eval)
    l[0] = 0
    l[1:] = np.cumsum(dl)
    
    newl = np.linspace(0,l[-1],n)
    good = np.r_[True,dl>0]
    newx = np.interp(newl, l[good], xup[good])
    
    return _eval_splines(splines, newx), newx
    
    
        

def unif_resample(x,n,weights,tol=.001,deg=3):    
    x = np.atleast_2d(x)
    weights = np.atleast_2d(weights)
    x = mu.remove_duplicate_rows(x)
    x_scaled = x * weights
    dl = mu.norms(x_scaled[1:] - x_scaled[:-1],1)
    l = np.cumsum(np.r_[0,dl])
    
    (tck,_) = si.splprep(x_scaled.T,k=deg,s = tol**2*len(x),u=l)
    
    newu = np.linspace(0,l[-1],n)
    out_scaled = np.array(si.splev(newu,tck)).T
    out = out_scaled/weights
    return out
    
    
def unif_resample4(Y,n,tol=.0001, deg=3):    
    Y = np.atleast_2d(Y)

    dl = mu.norms(Y[1:] - Y[:-1],1)
    l = np.cumsum(np.r_[0,dl])
    goodinds = np.r_[0,1+np.flatnonzero(dl > 1e-10)] 
    
    splines = [si.UnivariateSpline(l[goodinds], y[goodinds], s = tol**2*len(l)) for y in Y.T]
    
    def _eval_splines(_splines, _x):
        return np.array([spline(_x) for spline in splines]).T

    newl = np.linspace(0,l[-1],n)
    newx = np.interp(newl, l[goodinds], goodinds)
    return _eval_splines(splines, newl), newx
        
    
def unif_resample2(traj, max_diff, wt = None):        
    """
    Resample a trajectory so steps have same length in joint space    
    """
    import scipy.interpolate as si
    tol = .005
    if wt is not None: 
        wt = np.atleast_2d(wt)
        traj = traj*wt
        
        
    dl = mu.norms(traj[1:] - traj[:-1],1)
    l = np.cumsum(np.r_[0,dl])
    goodinds = np.r_[True, dl > 1e-8]
    deg = min(3, sum(goodinds) - 1)
    if deg < 1: return traj, np.arange(len(traj))
    
    nsteps = max(int(np.ceil(float(l[-1])/max_diff)),2)
    newl = np.linspace(0,l[-1],nsteps)

    ncols = traj.shape[1]
    colstep = 10
    traj_rs = np.empty((nsteps,ncols)) 
    for istart in xrange(0, traj.shape[1], colstep):
        (tck,_) = si.splprep(traj[goodinds, istart:istart+colstep].T,k=deg,s = tol**2*len(traj),u=l[goodinds])
        traj_rs[:,istart:istart+colstep] = np.array(si.splev(newl,tck)).T
    if wt is not None: traj_rs = traj_rs/wt

    newt = np.interp(newl, l, np.arange(len(traj)))

    return traj_rs, newt    

def test_resample():
    x = [0,0,0,1,2,3,4,4,4]
    t = range(len(x))
    inds = adaptive_resample(x, max_err = 1e-5)
    assert inds == [0, 2, 6, 8]
    inds = adaptive_resample(x, t=t, max_err = 0)
    assert inds == [0, 2, 6, 8]
    print "success"


    inds1 = fastrapp.resample(np.array(x)[:,None], t, 0, np.inf, np.inf)
    print inds1
    assert inds1.tolist() == [0,2,6,8]

def test_resample_big():
    from time import time
    t = np.linspace(0,1,1000)
    x0 = np.sin(t)[:,None]
    x = x0 + np.random.randn(len(x0), 50)*.1
    tstart = time()
    inds0 = adaptive_resample(x, t=t, max_err = .05, max_dt = .1)
    print time() - tstart, "seconds"

    print "now doing cpp version"
    tstart = time()
    inds1 = fastrapp.resample(x, t, .05, np.inf, .1)
    print time() - tstart, "seconds"
    
    assert np.allclose(inds0, inds1)

def interp_quats(newtimes, oldtimes, oldquats):
    "should actually do slerp"
    quats_unnormed = mu.interp2d(newtimes, oldtimes, oldquats)
    return mu.normr(quats_unnormed)
        

def interp_hmats(newtimes, oldtimes, oldhmats):
    oldposes = openravepy.poseFromMatrices(oldhmats)
    newposes = np.empty((len(newtimes), 7))
    newposes[:,4:7] = mu.interp2d(newtimes, oldtimes, oldposes[:,4:7])
    newposes[:,0:4] = interp_quats(newtimes, oldtimes, oldposes[:,0:4])
    newhmats = np.array(openravepy.matrixFromPoses(newposes))
    return newhmats
    
def test_resample3():
    x0 = np.random.randn(10,1)**2
    x0 = np.repeat(x0,10,axis=0)
    t0 = np.arange(len(x0))
    
    # t0 = np.repeat(t0,10,axis=0)
    
    x2, t2 = unif_resample2(x0,.1)
    x3, t3 = unif_resample3(x0,100)
    x4, t4 = unif_resample4(x0,100)
    import matplotlib.pyplot as plt
    
    print "uniformity:"
    print 2,np.var(mu.norms(x2[1:] - x2[:-1],1))
    print 3,np.var(mu.norms(x3[1:] - x3[:-1],1))
    print 4, np.var(mu.norms(x4[1:] - x4[:-1],1))
    plt.plot(t0, x0[:,0],'x-',ms=10)
    plt.plot(t2,x2[:,0],'.')
    plt.plot(t3,x3[:,0],'.')
    plt.plot(t4,x4[:,0],'.')
    plt.legend(['orig','2','3','4'])
    plt.show()
    
    

if __name__ == "__main__":
    test_resample3()
    # test_resample()
    # test_resample_big()