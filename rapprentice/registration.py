"""
Register point clouds to each other
"""

from __future__ import division
import numpy as np
import scipy.spatial.distance as ssd
from rapprentice import tps, svds, math_utils
# from svds import svds


"""
arrays are named like name_abc
abc are subscripts and indicate the what that tensor index refers to

index name conventions:
    m: test point index
    n: training point index
    a: input coordinate
    g: output coordinate
    d: gripper coordinate
"""

class Transformation(object):
    """
    Object oriented interface for transformations R^d -> R^d
    """
    def transform_points(self, x_ma):
        raise NotImplementedError
    def compute_jacobian(self, x_ma):
        raise NotImplementedError        

        
    def transform_bases(self, x_ma, rot_mad, orthogonalize=True, orth_method = "cross"):
        """
        orthogonalize: none, svd, qr
        """

        grad_mga = self.compute_jacobian(x_ma)
        newrot_mgd = np.array([grad_ga.dot(rot_ad) for (grad_ga, rot_ad) in zip(grad_mga, rot_mad)])
        

        if orthogonalize:
            if orth_method == "qr": 
                newrot_mgd =  orthogonalize3_qr(newrot_mgd)
            elif orth_method == "svd":
                newrot_mgd = orthogonalize3_svd(newrot_mgd)
            elif orth_method == "cross":
                newrot_mgd = orthogonalize3_cross(newrot_mgd)
            else: raise Exception("unknown orthogonalization method %s"%orthogonalize)
        return newrot_mgd
        
    def transform_hmats(self, hmat_mAD):
        """
        Transform (D+1) x (D+1) homogenius matrices
        """
        hmat_mGD = np.empty_like(hmat_mAD)
        hmat_mGD[:,:3,3] = self.transform_points(hmat_mAD[:,:3,3])
        hmat_mGD[:,:3,:3] = self.transform_bases(hmat_mAD[:,:3,3], hmat_mAD[:,:3,:3])
        hmat_mgd[:,3,:] = np.array([0,0,0,1])
        return hmat_mGD
        
    def compute_numerical_jacobian(self, x_d, epsilon=0.0001):
        "numerical jacobian"
        x0 = np.asfarray(x_d)
        f0 = self.transform_points(x0)
        jac = np.zeros(len(x0), len(f0))
        dx = np.zeros(len(x0))
        for i in range(len(x0)):
            dx[i] = epsilon
            jac[i] = (self.transform_points(x0+dx) - f0) / epsilon
            dx[i] = 0.
        return jac.transpose()

class ThinPlateSpline(Transformation):
    """
    members:
        x_na: centers of basis functions
        w_ng: 
        lin_ag: transpose of linear part, so you take x_na.dot(lin_ag)
        trans_g: translation part
    
    """
    def __init__(self, d=3):
        "initialize as identity"
        self.x_na = np.zeros((0,d))
        self.lin_ag = np.eye(d)
        self.trans_g = np.zeros(d)
        self.w_ng = np.zeros((0,d))

    def transform_points(self, x_ma):
        y_ng = tps.tps_eval(x_ma, self.lin_ag, self.trans_g, self.w_ng, self.x_na)
        return y_ng
    def compute_jacobian(self, x_ma):
        grad_mga = tps.tps_grad(x_ma, self.lin_ag, self.trans_g, self.w_ng, self.x_na)
        return grad_mga
        

def fit_ThinPlateSpline(x_na, y_ng, bend_coef=.1, rot_coef = 1e-5, wt_n=None):
    """
    x_na: source cloud
    y_nd: target cloud
    smoothing: penalize non-affine part
    angular_spring: penalize rotation
    wt_n: weight the points        
    """
    f = ThinPlateSpline()
    f.lin_ag, f.trans_g, f.w_ng = tps.tps_fit3(x_na, y_ng, bend_coef, rot_coef, wt_n)
    f.x_na = x_na
    return f        

def fit_ThinPlateSpline_RotReg(x_na, y_ng, bend_coef = .1, rot_coefs = (0.01,0.01,0.0025),scale_coef=.01):
    import fastrapp
    f = ThinPlateSpline()
    rfunc = fastrapp.rot_reg
    fastrapp.set_coeffs(rot_coefs, scale_coef)
    f.lin_ag, f.trans_g, f.w_ng = tps.tps_fit_regrot(x_na, y_ng, bend_coef, rfunc)
    f.x_na = x_na
    return f        
    
    

def loglinspace(a,b,n):
    "n numbers between a to b (inclusive) with constant ratio between consecutive numbers"
    return np.exp(np.linspace(np.log(a),np.log(b),n))    


def tps_rpm(x_nd, y_md, n_iter = 20, reg_init = .1, reg_final = .001, rad_init = .05, rad_final = .001, 
            plotting = False, verbose=True, f_init = None, return_full = False, plot_cb = None):
    """
    tps-rpm algorithm mostly as described by chui and rangaran
    reg_init/reg_final: regularization on curvature
    rad_init/rad_final: radius for correspondence calculation (meters)
    plotting: 0 means don't plot. integer n means plot every n iterations
    """
    _,d=x_nd.shape
    regs = loglinspace(reg_init, reg_final, n_iter)
    rads = loglinspace(rad_init, rad_final, n_iter)
    if f_init is not None: 
        f = f_init  
    else:
        f = ThinPlateSpline(d)
        f.trans_g = np.median(y_md,axis=0) - np.median(x_nd,axis=0)
    for i in xrange(n_iter):
        xwarped_nd = f.transform_points(x_nd)
        # targ_nd = find_targets(x_nd, y_md, corr_opts = dict(r = rads[i], p = .1))
        corr_nm = calc_correspondence_matrix(xwarped_nd, y_md, r=rads[i], p=.2, ratio_err_tol=1e-2,max_iter=10)

        wt_n = corr_nm.sum(axis=1)

        goodn = wt_n > .1

        targ_Nd = np.dot(corr_nm[goodn, :]/wt_n[goodn][:,None], y_md)
        
        if plotting and i%plotting==0:
            plot_cb(x_nd, y_md, targ_Nd, corr_nm, wt_n, f)
        
        
        x_Nd = x_nd[goodn]
        f = fit_ThinPlateSpline(x_Nd, targ_Nd, bend_coef = regs[i], wt_n = wt_n[goodn], rot_coef = 10*regs[i])


    if return_full:
        info = {}
        info["corr_nm"] = corr_nm
        info["goodn"] = goodn
        info["x_Nd"] = x_nd[goodn,:]
        info["targ_Nd"] = targ_Nd
        info["wt_N"] = wt_n[goodn]
        info["cost"] = tps.tps_cost(f.lin_ag, f.trans_g, f.w_ng, x_Nd, targ_Nd, regs[-1])
        return f, info
    else:
        return f

def logmap(m):
    "http://en.wikipedia.org/wiki/Axis_angle#Log_map_from_SO.283.29_to_so.283.29"
    theta = np.arccos(np.clip((np.trace(m) - 1)/2,-1,1))
    return (1/(2*np.sin(theta))) * np.array([[m[2,1] - m[1,2], m[0,2]-m[2,0], m[1,0]-m[0,1]]]), theta
# 
# def tps_rpm_zrot(x_nd, y_md, n_iter = 5, reg_init = .1, reg_final = .001, rad_init = .2, rad_final = .001, plotting = False, 
#                  verbose=True, rot_param=(.05, .05, .05), scale_param = .01, plot_cb = None):
#     """
#     Do tps_rpm algorithm for each z angle rotation
#     Then don't reestimate affine part in tps optimization
#     
#     rot param: meters error per point per radian
#     scale param: meters error per log2 scaling
#     
#     """
#     
#     n_initializations = 5
#     
#     n,d = x_nd.shape
#     regs = loglinspace(reg_init, reg_final, n_iter)
#     rads = loglinspace(rad_init, rad_final, n_iter)
#     zrots = np.linspace(-np.pi/2, pi/2, n_initializations)
# 
#     costs,tpscosts,regcosts = [],[],[]
#     fs = []
#     
#     # convert in to the right units: meters/pt -> meters*2
#     rot_coefs = np.array(rot_param)
#     scale_coef = scale_param
#     import fastmath
#     fastmath.set_coeffs(rot_coefs, scale_coef)
#     #def regfunc(b):        
#         #if np.linalg.det(b) < 0 or np.isnan(b).any(): return np.inf
#         #b = b.T
#         #u,s,vh = np.linalg.svd(b)
#         ##p = vh.T.dot(s.dot(vh))        
#         #return np.abs(np.log(s)).sum()*scale_coef + float(np.abs(logmap(u.dot(vh))).dot(rot_coefs))
#     regfunc = fastmath.rot_reg
#     reggrad = fastmath.rot_reg_grad
#     
#     for a in zrots:
#         f_init = ThinPlateSplineRegularizedLinearPart(regfunc, reggrad=reggrad)
#         f_init.n_fit_iters = 2
#         f_init.lin_ag[:2,:2] = np.array([[cos(a), sin(a)],[-sin(a), cos(a)]])
#         f_init.trans_g =  y_md.mean(axis=0) - f_init.transform_points(x_nd).mean(axis=0)
#         f, info = tps_rpm(x_nd, y_md, n_iter=n_iter, reg_init=reg_init, reg_final=reg_final, rad_init = rad_init, rad_final = rad_final, plotting=plotting, verbose=verbose, f_init=f_init, return_full=True, plot_cb=plot_cb)
#         ypred_ng = f.transform_points(x_nd)
#         dists_nm = ssd.cdist(ypred_ng, y_md)
#         # how many radians rotation is one mm average error reduction worth?
# 
#         tpscost = info["cost"]
#         # seems like a reasonable goodness-of-fit measure
#         regcost = regfunc(f.lin_ag)
#         tpscosts.append(dists_nm.min(axis=1).mean())
#         regcosts.append(regcost)
#         costs.append(tpscost + regcost)
#         fs.append(f)        
#         print "linear part", f.lin_ag
#         u,s,vh = np.linalg.svd(f.lin_ag)
#         print "angle-axis:",logmap(u.dot(vh))
#         print "scaling:", s
#         
#     print "zrot | tps | reg | total"
#     for i in xrange(len(zrots)):
#         print "%.5f | %.5f | %.5f | %.5f"%(zrots[i], tpscosts[i], regcosts[i], costs[i])
# 
#     i_best = np.argmin(costs)
#     best_f = fs[i_best]
#     print "best initialization angle", zrots[i_best]*180/np.pi
#     u,s,vh = np.linalg.svd(best_f.lin_ag)
#     print "best rotation axis,angle:",logmap(u.dot(vh))
#     print "best scaling:", s
# 
#     if plotting:
#         plot_cb(x_nd, y_md, None, None, None, f)
# 
#     return best_f


def find_targets(x_md, y_nd, corr_opts):
    """finds correspondence matrix, and then for each point in source cloud,
    find the weighted average of its "partners" in the target cloud"""

    corr_mn = calc_correspondence_matrix(x_md, y_nd, **corr_opts)
    # corr_mn = M.match(x_md, y_nd)
    # corr_mn = corr_mn / corr_mn.sum(axis=1)[:,None]
    return np.dot(corr_mn, y_nd)        

def calc_correspondence_matrix(x_nd, y_md, r, p, max_iter=20, ratio_err_tol=1e-3):
    """
    sinkhorn procedure. see tps-rpm paper
    """
    n = x_nd.shape[0]
    m = y_md.shape[0]
    dist_nm = ssd.cdist(x_nd, y_md,'euclidean')
    prob_nm = np.exp(-dist_nm / r)
    prob_nm_orig = prob_nm.copy()
    pnoverm = (float(p)*float(n)/float(m))
    for _ in xrange(max_iter):
        colsums = pnoverm + prob_nm.sum(axis=0)        
        prob_nm /=  + colsums[None,:]
        rowsums = p + prob_nm.sum(axis=1)
        prob_nm /= rowsums[:,None]
        
        if ((rowsums-1).__abs__() < ratio_err_tol).all() and ((colsums-1).__abs__() < ratio_err_tol).all():
            break

    prob_nm = np.sqrt(prob_nm_orig * prob_nm)
    prob_nm /= (p + prob_nm.sum(axis=1))[:,None] # rows sum to 1

    return prob_nm
    
def calc_correspondence_matrix2(x_nd, y_md, r, p, max_iter=20, ratio_err_tol=1e-3):
    from scipy.weave import inline
    code = """
    int N,D,M,i,n,m;
    double dev;
    
    N = Nx_nd[0];
    D = Nx_nd[1];
    M = Ny_md[0];

    for (i=0; i < max_iter; ++i) {


        for (m=0; m < M; ++m) COLSUMS1(m) = ((float)p*N)/M;
        for (n=0; n < N; ++n) {
            for (m=0; m < M; ++m) {
                COLSUMS1(m) += PROB_NM2(n,m);
            }            
        }
        
        
        for (n=0; n < N; ++n) {
            for (m=0; m < M; ++m) {
                PROB_NM2(n,m) /= COLSUMS1(m);
            }            
        }
        
        for (n=0; n < N; ++n) rowsums[n] = p;
        for (n=0; n < N; ++n) {
            for (m=0; m < M; ++m) {
                ROWSUMS1(n) += PROB_NM2(n,m);
            }            
        }
        for (n=0; n < N; ++n) {
            for (m=0; m < M; ++m) {
                PROB_NM2(n,m) /= ROWSUMS1(n);
            }            
        }
        
        dev=0; 
        for (int n=0; n < N; ++n) dev=fmax(dev, fabs(ROWSUMS1(n)-1));
        for (int m=0; m < M; ++m) dev=fmax(dev, fabs(COLSUMS1(m)-1));
        if (dev < ratio_err_tol) {
//            printf("done after %i iters\\n",i);
            break;
        }
        
    }
    

    """
    dist_nm = ssd.cdist(x_nd, y_md,'euclidean')
    prob_nm = np.exp(-dist_nm / r)
    rowsums = np.zeros(dist_nm.shape[0])
    colsums = np.zeros(dist_nm.shape[1])
    p = float(p)
    prob_nm_orig = prob_nm.copy()    
    inline(code,["x_nd","y_md","rowsums","colsums","prob_nm","p","max_iter","ratio_err_tol"])

    prob_nm = np.sqrt(prob_nm_orig * prob_nm)
    prob_nm /= (p + prob_nm.sum(axis=1))[:,None] # rows sum to 1
    
    return prob_nm

def nan2zero(x):
    np.putmask(x, np.isnan(x), 0)
    return x


def fit_score(src, targ, dist_param):
    "how good of a partial match is src to targ"
    sqdists = ssd.cdist(src, targ,'sqeuclidean')
    return -np.exp(-sqdists/dist_param**2).sum()

def orthogonalize3_cross(mats_n33):
    "turns each matrix into a rotation"

    x_n3 = mats_n33[:,:,0]
    y_n3 = mats_n33[:,:,1]
    # z_n3 = mats_n33[:,:,2]

    xnew_n3 = math_utils.normr(x_n3)
    znew_n3 = math_utils.normr(np.cross(xnew_n3, y_n3))
    ynew_n3 = math_utils.normr(np.cross(znew_n3, xnew_n3))

    return np.concatenate([xnew_n3[:,:,None], ynew_n3[:,:,None], znew_n3[:,:,None]],2)

def orthogonalize3_svd(x_k33):
    u_k33, _s_k3, v_k33 = svds.svds(x_k33)
    return (u_k33[:,:,:,None] * v_k33[:,None,:,:]).sum(axis=2)

def orthogonalize3_qr(_x_k33):
    raise NotImplementedError