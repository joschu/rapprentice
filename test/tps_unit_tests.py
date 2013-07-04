
from rapprentice import testing, tps
import numpy as np

@testing.testme
def nonrigidity_gradient():
    import numdifftools as ndt
    D = 3
    pts0 = np.random.randn(10,D)
    pts_eval = np.random.randn(3,D)
    def err_partial(params):
        lin_ag = params[0:9].reshape(3,3)
        trans_g = params[9:12]
        w_ng = params[12:].reshape(-1,3)
        return tps.tps_nr_err(pts_eval, lin_ag, trans_g, w_ng, pts0)    
    lin_ag, trans_g, w_ng = np.random.randn(D,D), np.random.randn(D), np.random.randn(len(pts0), D)
    J1 = tps.tps_nr_grad(pts_eval, lin_ag, trans_g, w_ng, pts0)
    J = ndt.Jacobian(err_partial)(np.r_[lin_ag.flatten(), trans_g.flatten(), w_ng.flatten()])
    assert np.allclose(J1, J)

@testing.testme
def jacobian_of_tps():
    import numdifftools as ndt
    D = 3
    pts0 = np.random.randn(100,3)
    pts_eval = np.random.randn(20,D)
    lin_ag, trans_g, w_ng, x_na = np.random.randn(D,D), np.random.randn(D), np.random.randn(len(pts0), D), pts0    
    def eval_partial(x_ma_flat):
        x_ma = x_ma_flat.reshape(-1,3)
        return tps.tps_eval(x_ma, lin_ag, trans_g, w_ng, pts0)
    for i in xrange(len(pts_eval)):
        rots = ndt.Jacobian(eval_partial)(pts_eval[i])
        rots1 = tps.tps_grad(pts_eval[i:i+1], lin_ag, trans_g, w_ng, pts0)
        assert np.allclose(rots1, rots)

@testing.testme
def fitting_methods_equivalent():
    pts0 = np.random.randn(200,3)
    pts1 = np.random.randn(200,3)
    bend_coef = 13
    lin_ag, trans_g, w_ng = tps.tps_fit(pts0, pts1, bend_coef, 0)    
    lin2_ag, trans2_g, w2_ng = tps.tps_fit2(pts0, pts1, bend_coef, 0)
    lin3_ag, trans3_g, w3_ng = tps.tps_fit3(pts0, pts1, bend_coef, 0, np.ones(len(pts0)))

    assert np.allclose(lin_ag, lin2_ag)
    assert np.allclose(trans_g, trans2_g)
    assert np.allclose(w_ng, w2_ng)

    assert np.allclose(lin_ag, lin3_ag)
    assert np.allclose(trans_g, trans3_g)
    assert np.allclose(w_ng, w3_ng)


    lin2_ag, trans2_g, w2_ng = tps.tps_fit2(pts0, pts1, bend_coef, .01)
    lin3_ag, trans3_g, w3_ng = tps.tps_fit3(pts0, pts1, bend_coef, .01, np.ones(len(pts0)))

    assert np.allclose(lin2_ag, lin3_ag)
    assert np.allclose(trans2_g, trans3_g)
    assert np.allclose(w2_ng, w3_ng)


# @testing.testme
def check_that_nr_fit_runs():
    from jds_image_proc.clouds import voxel_downsample
    #from brett2.ros_utils import RvizWrapper    
    #import lfd.registration as lr
    ##import lfd.warping as lw    
    #if rospy.get_name() == "/unnamed":
        #rospy.init_node("test_rigidity", disable_signals=True)
        #from time import sleep
        #sleep(1)
    #rviz = RvizWrapper.create()
    
    pts0 = np.loadtxt("../test/rope_control_points.txt")
    pts1 = np.loadtxt("../test/bad_rope.txt")    
    pts_rigid = voxel_downsample(pts0[:10], .02)
    #lr.Globals.setup()
    np.seterr(all='ignore')
    np.set_printoptions(suppress=True)

    lin_ag, trans_g, w_eg, x_ea = tps.tps_nr_fit_enhanced(pts0, pts1, 0.01, pts_rigid, 0.001, method="newton",plotting=0)
    #lin_ag2, trans_g2, w_ng2 = tps_fit(pts0, pts1, .01, .01)
    #assert np.allclose(w_ng, w_ng2)
    def eval_partial(x_ma):
        return tps_eval(x_ma, lin_ag, trans_g, w_eg, x_ea) 
    #lr.plot_orig_and_warped_clouds(eval_partial, pts0, pts1, res=.008)
    #handles = lw.draw_grid(rviz, eval_partial, pts0.min(axis=0), pts0.max(axis=0), 'base_footprint')

    grads = tps.tps_grad(pts_rigid, lin_ag, trans_g, w_eg, x_ea)
    #print "worst violation:",np.max(np.abs([grad.T.dot(grad)-np.eye(3) for grad in grads]))

@testing.testme
def tps_fit_fixedrot_is_minimizer():
    """
    check that tps_fit_fixedrot minimizes tps_cost subject to constraint
    """
    x_na = np.random.randn(100,3)
    y_ng = np.random.randn(100,3)
    bend_coef = .1
    lin_ag = np.random.randn(3,3)
    trans_g, w_ng = tps.tps_fit_fixedrot(x_na, y_ng, bend_coef, lin_ag)
    hopefully_min_cost = tps.tps_cost(lin_ag, trans_g, w_ng, x_na, y_ng, bend_coef)
    
    n_tries = 50
    other_costs = np.empty(n_tries)
    for i in xrange(n_tries):
        N = len(x_na)
        _u,_s,_vh = np.linalg.svd(np.c_[x_na, np.ones((N,1))], full_matrices=True)
        pert = .01*_u[:,4:].dot(np.random.randn(N-4,3))
        assert np.allclose(x_na.T.dot(pert),np.zeros((3,3)))
        other_costs[i] = tps.tps_cost(lin_ag, trans_g, w_ng+pert, x_na, y_ng, bend_coef)
    assert (other_costs > hopefully_min_cost).all()
    

@testing.testme
def tps_fit_is_minimizer():
    """
    check that tps_fit_fixedrot minimizes tps_cost subject to constraint
    """
    x_na = np.random.randn(100,3)
    y_ng = np.random.randn(100,3)
    bend_coef = 10
    lin_ag = np.random.randn(3,3)
    trans_g, w_ng = tps.tps_fit_fixedrot(x_na, y_ng, bend_coef, lin_ag)
    hopefully_min_cost = tps.tps_cost(lin_ag, trans_g, w_ng, x_na, y_ng, bend_coef)
    
    n_tries = 50
    other_costs = np.empty(n_tries)
    N = len(x_na)
    _u,_s,_vh = np.linalg.svd(np.c_[x_na, np.ones((N,1))], full_matrices=True)
    for i in xrange(n_tries):
        pert = .01*_u[:,4:].dot(np.random.randn(N-4,3))
        assert np.allclose(x_na.T.dot(pert),np.zeros((3,3)))
        assert np.allclose(pert.sum(axis=0),np.zeros(3))
        other_costs[i] = tps.tps_cost(lin_ag, trans_g, w_ng+pert, x_na, y_ng, bend_coef)
    assert (other_costs > hopefully_min_cost).all()
        
    
@testing.testme
def tps_regrot_with_quad_cost():
    x_na = np.random.randn(100,3)
    y_ng = np.random.randn(100,3)
    bend_coef = 10
    rot_coef = 19
    # wt_n = np.random.rand(len(x_na))
    def rfunc(b):
        return rot_coef*((b - np.eye(3))**2).sum()
    correct_lin_ag, correct_trans_g, correct_w_ng = tps.tps_fit2(x_na, y_ng, bend_coef, rot_coef)
    lin_ag, trans_g, w_ng = tps.tps_fit_regrot(x_na, y_ng, bend_coef, rfunc, max_iter=30)
    assert np.allclose(correct_trans_g, trans_g, atol=1e-2)    
    assert np.allclose(correct_lin_ag, lin_ag, atol=1e-2)
    assert np.allclose(correct_w_ng, w_ng,atol=1e-2)
    
@testing.testme
def rot_reg_works():
    from numpy import sin,cos
    from rapprentice import registration
    import fastrapp
    x = np.random.randn(100,3)
    a = .1
    R = np.array([[cos(a), sin(a),0],[-sin(a), cos(a), 0],[0,0,1]])
    y = x.dot(R.T)
    f = registration.fit_ThinPlateSpline_RotReg(x, y, bend_coef = .1, rot_coefs = [.1,.1,0], scale_coef = 1)
    assert np.allclose(R.T, f.lin_ag, atol = 1e-4)
    

if __name__ == "__main__":
    # tps.VERBOSE = True
    tps.ENABLE_SLOW_TESTS=True
    np.seterr(all='ignore')
    testing.test_all()
    # tps_regrot_with_quad_cost()
    # fitting_methods_equivalent()
