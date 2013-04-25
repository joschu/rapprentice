"""
Resample time serieses to reduce the number of datapoints
"""
from __future__ import division
import numpy as np
from rapprentice import log
import fastrapp

def lerp(a, b, fracs):
    "linearly interpolate between a and b"
    fracs = fracs[:,None]
    return a*(1-fracs) + b*fracs

def adaptive_resample(x, t = None, max_err = np.inf, max_dx = np.inf, max_dt = np.inf, normfunc = None):
    #return np.arange(0, len(x), 100)
    log.info("resampling a path of length %i"%len(x))
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
if __name__ == "__main__":
    test_resample()
    test_resample_big()